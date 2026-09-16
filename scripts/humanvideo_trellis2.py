#!/usr/bin/env python3
"""Linux TRELLIS.2 object reconstruction; explicit object masks/boxes, no tracking."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
import traceback

ROOT = Path(__file__).resolve().parents[1]
IMAGES = {'.jpg', '.jpeg', '.png', '.webp', '.bmp'}
VIDEOS = {'.mp4', '.mov', '.avi', '.mkv', '.webm'}


def save_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')
    tmp.replace(path)


def download(args):
    from huggingface_hub import snapshot_download
    # Keep the HF cache layout: upstream resolves both relative and cross-repo weights.
    jobs = [
        ('microsoft/TRELLIS.2-4B', ['pipeline.json', 'ckpts/*']),
        ('microsoft/TRELLIS-image-large', ['ckpts/ss_dec_conv3d_16l8_fp16.*']),
        ('facebook/dinov3-vitl16-pretrain-lvd1689m', None),
        ('briaai/RMBG-2.0', None),
    ]
    records = []
    for repo, patterns in jobs:
        print(f'Downloading {repo}', flush=True)
        location = snapshot_download(repo_id=repo, allow_patterns=patterns)
        records.append({'repo': repo, 'snapshot': location})
    save_json(args.record, records)
    print(f'Download complete: {args.record}')


def extract(args):
    import cv2
    from PIL import Image
    source = args.input.resolve()
    output = args.output.resolve()
    if not source.exists():
        raise ValueError(f'Input does not exist: {source}')
    if source.is_dir() and (output == source or source in output.parents):
        raise ValueError('Keep frame output outside the input directory.')
    files = sorted(source.rglob('*')) if source.is_dir() else [source]
    rows = []
    for path in files:
        if path.suffix.lower() not in IMAGES | VIDEOS:
            continue
        key = re.sub(r'[^A-Za-z0-9_-]', '_', path.stem) + '_' + hashlib.sha256(str(path).encode()).hexdigest()[:10]
        dest = output / key
        dest.mkdir(parents=True, exist_ok=True)
        if path.suffix.lower() in IMAGES:
            with Image.open(path) as im:
                im.save(dest / 'image.png')
            rows.append({'source': str(path), 'image': str(dest / 'image.png')})
            continue
        cap = cv2.VideoCapture(str(path))
        try:
            fps = cap.get(cv2.CAP_PROP_FPS)
            count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            if not cap.isOpened() or not math.isfinite(fps) or fps <= 0 or count <= 0:
                raise ValueError(f'Cannot read video metadata: {path}')
            # Uniformly cover the video, with a bounded number of decoded frames.
            n = min(args.max_frames, max(1, math.ceil(count / fps / args.every)))
            indices = sorted({round(i * (count - 1) / max(1, n - 1)) for i in range(n)})
            for index in indices:
                cap.set(cv2.CAP_PROP_POS_FRAMES, index)
                ok, frame = cap.read()
                if not ok:
                    raise ValueError(f'Frame decode failed: {path}, frame {index}')
                target = dest / f'frame_{index:08d}.png'
                if not cv2.imwrite(str(target), frame):
                    raise IOError(f'Cannot write {target}')
                rows.append({'source': str(path), 'frame': index, 'seconds': index / fps, 'image': str(target)})
        finally:
            cap.release()
    if not rows:
        raise ValueError('No supported images/videos found.')
    save_json(output / 'frames.json', rows)
    print(f'Extracted {len(rows)} images. Inspect them and create an object manifest.')


def object_image(row, base):
    import cv2
    import numpy as np
    from PIL import Image
    with Image.open(base / row['image']) as source:
        rgba = np.array(source.convert('RGBA'))
    h, w = rgba.shape[:2]
    if row.get('mask'):
        with Image.open(base / row['mask']) as source:
            mask = np.array(source.convert('L')) > 127
        if mask.shape != (h, w):
            raise ValueError('Object mask must match the full input image dimensions.')
    elif row.get('bbox'):
        box = row['bbox']
        if len(box) != 4 or any(type(v) is not int for v in box):
            raise ValueError('bbox must be four integer pixel coordinates [x1,y1,x2,y2].')
        x1, y1, x2, y2 = box
        if not (0 <= x1 < x2 <= w and 0 <= y1 < y2 <= h):
            raise ValueError('bbox is outside the image or empty.')
        if x1 == y1 == 0 and x2 == w and y2 == h:
            raise ValueError('GrabCut box must leave some background outside it.')
        labels = np.zeros((h, w), dtype=np.uint8)
        cv2.grabCut(rgba[:, :, :3].copy(), labels, (x1, y1, x2-x1, y2-y1),
                    np.zeros((1, 65)), np.zeros((1, 65)), 5, cv2.GC_INIT_WITH_RECT)
        mask = (labels == cv2.GC_FGD) | (labels == cv2.GC_PR_FGD)
    elif np.any(rgba[:, :, 3] < 255):
        mask = rgba[:, :, 3] > 127
    else:
        raise ValueError('Supply an object-only mask, bbox, or transparent object PNG.')
    if row.get('exclude_mask'):
        with Image.open(base / row['exclude_mask']) as source:
            exclude = np.array(source.convert('L')) > 127
        if exclude.shape != (h, w):
            raise ValueError('exclude_mask must match the input dimensions.')
        mask &= ~exclude
    ys, xs = np.where(mask)
    if len(xs) < 64 or xs.max() == xs.min() or ys.max() == ys.min():
        raise ValueError('Object mask is empty or too small.')
    rgba[:, :, 3] = mask.astype(np.uint8) * 255
    crop = Image.fromarray(rgba).crop((int(xs.min()), int(ys.min()), int(xs.max()+1), int(ys.max()+1)))
    size = math.ceil(max(crop.size) * 1.15)
    canvas = Image.new('RGBA', (size, size), (0, 0, 0, 0))
    canvas.paste(crop, ((size-crop.width)//2, (size-crop.height)//2))
    return canvas


def reconstruct(args):
    rows = json.loads(args.manifest.read_text(encoding='utf-8'))
    if not isinstance(rows, list) or not rows:
        raise ValueError('Manifest must be a nonempty JSON list.')
    ids = [r['id'] for r in rows]
    if len(set(ids)) != len(ids) or any(not re.fullmatch(r'[A-Za-z0-9_-]+', v) for v in ids):
        raise ValueError('IDs must be unique and contain only ASCII letters, digits, _ or -.')
    pipeline = None
    failures = 0
    for row in rows:
        dest = args.output / row['id']
        dest.mkdir(parents=True, exist_ok=True)
        # Never silently reuse results after masks, input, or generation settings change.
        if (dest / 'object.glb').exists():
            raise FileExistsError(f'{dest}/object.glb exists; choose a new output directory.')
        metadata = {'input': row, 'seed': args.seed, 'pipeline': args.pipeline,
                    'status': 'preparing', 'metric_scale': False}
        try:
            image = object_image(row, args.manifest.resolve().parent)
            image.save(dest / 'object.png')
            if args.prepare_only:
                metadata['status'] = 'prepared'
                continue
            if pipeline is None:
                os.environ.setdefault('CUDA_VISIBLE_DEVICES', '0')
                os.environ.setdefault('ATTN_BACKEND', 'xformers')
                os.environ.setdefault('SPARSE_ATTN_BACKEND', 'xformers')
                os.environ.setdefault('TORCH_EXTENSIONS_DIR', str(ROOT / 'tmp/torch_extensions_cu129'))
                os.environ.setdefault('PYTORCH_CUDA_ALLOC_CONF', 'expandable_segments:True')
                sys.path.insert(0, str(args.repo.resolve()))
                import torch
                from trellis2_attention import configure_attention
                configure_attention()
                import o_voxel
                from trellis2.pipelines import Trellis2ImageTo3DPipeline
                if not torch.cuda.is_available():
                    raise RuntimeError('CUDA GPU is required for reconstruction.')
                pipeline = Trellis2ImageTo3DPipeline.from_pretrained('microsoft/TRELLIS.2-4B')
                pipeline.cuda()
            mesh = pipeline.run(image, seed=args.seed, pipeline_type=args.pipeline)[0]
            mesh.simplify(16777216)
            glb = o_voxel.postprocess.to_glb(
                vertices=mesh.vertices, faces=mesh.faces, attr_volume=mesh.attrs,
                coords=mesh.coords, attr_layout=mesh.layout, voxel_size=mesh.voxel_size,
                aabb=[[-0.5, -0.5, -0.5], [0.5, 0.5, 0.5]],
                decimation_target=args.faces, texture_size=args.texture_size,
                remesh=True, remesh_band=1, remesh_project=0, verbose=True)
            temp = dest / 'object.partial.glb'
            glb.export(str(temp), extension_webp=True)
            temp.replace(dest / 'object.glb')
            metadata['status'] = 'complete'
            del mesh, glb
            torch.cuda.empty_cache()
        except Exception as exc:
            failures += 1
            metadata.update(status='failed', error=str(exc))
            traceback.print_exc()
        finally:
            save_json(dest / 'result.json', metadata)
    if failures:
        raise SystemExit(f'{failures} object(s) failed; see result.json files.')


def positive(value):
    value = int(value)
    if value <= 0:
        raise argparse.ArgumentTypeError('Must be positive.')
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    p = sub.add_parser('download', help='Cache TRELLIS.2 and all auxiliary model weights')
    p.add_argument('--record', type=Path, default=ROOT / 'model-downloads.json')
    p.set_defaults(func=download)
    p = sub.add_parser('extract', help='Copy images and uniformly sample videos')
    p.add_argument('--input', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--every', type=positive, default=2, help='Target seconds between frames')
    p.add_argument('--max-frames', type=positive, default=32, help='Maximum per video')
    p.set_defaults(func=extract)
    p = sub.add_parser('reconstruct', help='Prepare selected objects and generate GLBs')
    p.add_argument('--manifest', type=Path, required=True)
    p.add_argument('--output', type=Path, default=ROOT / 'outputs')
    p.add_argument('--repo', type=Path, default=Path(os.environ.get('TRELLIS2_DIR', ROOT / 'third_party/TRELLIS.2')))
    p.add_argument('--prepare-only', action='store_true', help='CPU-only object extraction preview')
    p.add_argument('--pipeline', choices=['512', '1024', '1024_cascade', '1536_cascade'], default='512')
    p.add_argument('--seed', type=int, default=42)
    p.add_argument('--faces', type=positive, default=100000)
    p.add_argument('--texture-size', type=positive, default=2048)
    p.set_defaults(func=reconstruct)
    args = parser.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
