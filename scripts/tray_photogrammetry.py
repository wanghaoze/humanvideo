"""Measured tray reconstruction. Python 3.10+, COLMAP; metric export needs trimesh."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess


def distance(pair):
    a, b = pair['a'], pair['b']
    if len(a) != 3 or len(b) != 3:
        raise ValueError('a and b must contain three coordinates')
    values = [*a, *b, pair['distance_mm']]
    if not all(math.isfinite(float(x)) for x in values):
        raise ValueError('Measurements must be finite')
    d = math.dist(a, b)
    if d <= 0 or pair['distance_mm'] <= 0:
        raise ValueError('Distances must be positive')
    return d


def fit_measurements(data):
    calibration = data['calibration']
    checks = data['validation']
    if not calibration or not checks:
        raise ValueError('Provide calibration AND independent validation measurements')
    keys = lambda p: tuple(sorted((tuple(p['a']), tuple(p['b']))))
    if {keys(p) for p in calibration} & {keys(p) for p in checks}:
        raise ValueError('Validation must not reuse calibration endpoint pairs')
    ds = [distance(p) for p in calibration]
    # Least-squares scale, real distances in mm and reconstruction distances arbitrary.
    scale_mm = sum(d * p['distance_mm'] for d, p in zip(ds, calibration)) / sum(d*d for d in ds)
    result = {'meters_per_input_unit': scale_mm / 1000}
    for name, pairs in [('calibration', calibration), ('validation', checks)]:
        rows = []
        for p in pairs:
            predicted = distance(p) * scale_mm
            rows.append({'name': p.get('name', ''), 'measured_mm': p['distance_mm'],
                         'predicted_mm': predicted, 'error_mm': predicted - p['distance_mm']})
        result[name] = rows
    errors = [abs(r['error_mm']) for r in result['validation']]
    result['validation_max_abs_mm'] = max(errors)
    result['validation_rmse_mm'] = math.sqrt(sum(e*e for e in errors) / len(errors))
    return result


def reconstruct(args):
    executable = shutil.which(args.colmap)
    if not executable:
        raise ValueError('COLMAP not found; install a CUDA-enabled build and add it to PATH')
    images = args.images.resolve()
    files = sorted(p for p in images.rglob('*') if p.suffix.lower() in {'.jpg', '.jpeg', '.png', '.tif', '.tiff'})
    if len(files) < 10:
        raise ValueError('Need at least 10 overlapping photos from ONE fixed-object session')
    out = args.output.resolve()
    if out == images or images in out.parents:
        raise ValueError('Output must be outside the image directory')
    if out.exists() and any(out.iterdir()):
        raise ValueError('Use a new empty output directory to avoid mixing reconstructions')
    out.mkdir(parents=True, exist_ok=True)
    sparse, dense = out / 'sparse', out / 'dense'
    sparse.mkdir()
    dense.mkdir()
    manifest = []
    # Stream hashes to support Python 3.10 and large original photographs.
    for p in files:
        h = hashlib.sha256()
        with p.open('rb') as f:
            for chunk in iter(lambda: f.read(1024*1024), b''):
                h.update(chunk)
        manifest.append({'file': str(p.relative_to(images)), 'sha256': h.hexdigest()})
    (out / 'inputs.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')

    def run(command, **options):
        cmd = [executable, command]
        for key, value in options.items():
            cmd.extend(['--' + key, str(value)])
        print(subprocess.list2cmdline(cmd), flush=True)
        with (out / 'commands.jsonl').open('a', encoding='utf-8') as log:
            log.write(json.dumps(cmd) + '\n')
        with (out / (command + '.log')).open('w', encoding='utf-8') as log:
            subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, check=True)

    db = out / 'database.db'
    run('feature_extractor', database_path=db, image_path=images,
        **{'ImageReader.single_camera': 1, 'ImageReader.camera_model': 'OPENCV'})
    run('exhaustive_matcher', database_path=db)
    run('mapper', database_path=db, image_path=images, output_path=sparse)
    models = [p for p in sparse.iterdir() if p.is_dir() and (p / 'cameras.bin').exists()]
    if len(models) != 1:
        raise ValueError(f'Expected one connected model, got {len(models)}. Inspect sparse models and improve overlap; no automatic model selection.')
    run('model_analyzer', path=models[0])
    run('image_undistorter', image_path=images, input_path=models[0], output_path=dense,
        output_type='COLMAP', max_image_size=args.max_image_size)
    run('patch_match_stereo', workspace_path=dense, workspace_format='COLMAP',
        **{'PatchMatchStereo.geom_consistency': 'true'})
    run('stereo_fusion', workspace_path=dense, workspace_format='COLMAP', input_type='geometric',
        output_path=dense / 'fused.ply')
    if args.mesh:
        run('poisson_mesher', input_path=dense / 'fused.ply', output_path=dense / 'mesh_candidate.ply')
    print(f'Finished: {dense}. Arbitrary scale; inspect coverage and calibrate before simulation.')


def metric(args):
    import trimesh
    data = json.loads(args.measurements.read_text(encoding='utf-8'))
    report = fit_measurements(data)
    report['tolerance_mm'] = args.tolerance_mm
    report['passed'] = report['validation_max_abs_mm'] <= args.tolerance_mm
    if args.output.exists():
        raise ValueError('Output already exists; choose a new path')
    if args.output.suffix.lower() != '.ply':
        raise ValueError('Use .ply output (mesh or point cloud)')
    obj = trimesh.load(str(args.input), process=False)
    if not isinstance(obj, (trimesh.Trimesh, trimesh.points.PointCloud)) or len(obj.vertices) == 0:
        raise ValueError('Input must be a nonempty single mesh or point cloud')
    obj.apply_scale(report['meters_per_input_unit'])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report_path = args.output.with_suffix('.report.json')
    if report_path.exists():
        raise ValueError('Report already exists; choose a new output path')
    report['source'] = str(args.input.resolve())
    report_path.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
    if not report['passed']:
        raise ValueError('Independent dimension checks failed. Report saved; no geometry exported.')
    obj.export(str(args.output))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    p = sub.add_parser('reconstruct')
    p.add_argument('--images', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--colmap', default='colmap')
    p.add_argument('--max-image-size', type=int, default=3200)
    p.add_argument('--mesh', action='store_true', help='Optional Poisson candidate; may bridge holes/thin walls')
    p.set_defaults(func=reconstruct)
    p = sub.add_parser('metric')
    p.add_argument('--input', type=Path, required=True)
    p.add_argument('--measurements', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--tolerance-mm', type=float, required=True)
    p.set_defaults(func=metric)
    args = parser.parse_args()
    if args.command == 'metric' and (not math.isfinite(args.tolerance_mm) or args.tolerance_mm <= 0):
        parser.error('--tolerance-mm must be finite and positive')
    if args.command == 'reconstruct' and args.max_image_size < 1:
        parser.error('--max-image-size must be positive')
    try:
        args.func(args)
    except (ValueError, OSError, subprocess.CalledProcessError) as exc:
        parser.exit(1, f'Error: {exc}\n')


if __name__ == '__main__':
    main()
