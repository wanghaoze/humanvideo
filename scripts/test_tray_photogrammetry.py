import unittest
import argparse
import json
import tempfile
from pathlib import Path
from tray_photogrammetry import fit_measurements


class MetricTests(unittest.TestCase):
    def data(self):
        return {'calibration': [{'a': [0,0,0], 'b': [2,0,0], 'distance_mm': 200}],
                'validation': [{'a': [0,0,0], 'b': [0,3,0], 'distance_mm': 301}]}

    def test_scale_and_held_out_error(self):
        r = fit_measurements(self.data())
        self.assertAlmostEqual(r['meters_per_input_unit'], 0.1)
        self.assertAlmostEqual(r['validation_max_abs_mm'], 1)
        self.assertAlmostEqual(r['validation'][0]['error_mm'], -1)

    def test_validation_does_not_change_scale(self):
        d = self.data()
        d['validation'][0]['distance_mm'] = 900
        self.assertAlmostEqual(fit_measurements(d)['meters_per_input_unit'], 0.1)

    def test_reused_pair_rejected(self):
        d = self.data()
        d['validation'] = [{'a': [2,0,0], 'b': [0,0,0], 'distance_mm': 200}]
        with self.assertRaises(ValueError):
            fit_measurements(d)

    def test_invalid_measurements_rejected(self):
        for value in [0, -1, float('nan'), float('inf')]:
            d = self.data()
            d['calibration'][0]['distance_mm'] = value
            with self.assertRaises(ValueError):
                fit_measurements(d)

    def test_ply_export_and_failure_gate(self):
        try:
            import trimesh
        except ImportError:
            self.skipTest('trimesh not installed')
        from tray_photogrammetry import metric
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / 'source.ply'
            trimesh.points.PointCloud([[0,0,0], [2,0,0], [0,3,0]]).export(str(source))
            measurements = root / 'measurements.json'
            measurements.write_text(json.dumps(self.data()), encoding='utf-8')
            args = argparse.Namespace(input=source, measurements=measurements,
                                      output=root / 'meters.ply', tolerance_mm=1.1)
            metric(args)
            result = trimesh.load(str(args.output), process=False)
            self.assertAlmostEqual(float(result.vertices[1][0]), 0.2, places=6)
            args.output = root / 'rejected.ply'
            args.tolerance_mm = 0.5
            with self.assertRaises(ValueError):
                metric(args)
            self.assertFalse(args.output.exists())
            self.assertTrue(args.output.with_suffix('.report.json').exists())


if __name__ == '__main__':
    unittest.main()
