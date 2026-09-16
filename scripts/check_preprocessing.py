"""CPU smoke checks using synthetic media; does not test GPU inference."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile

import cv2
import numpy as np
from PIL import Image

SCRIPT = Path(__file__).with_name('humanvideo_trellis2.py')
spec = importlib.util.spec_from_file_location('runner', SCRIPT)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)

with tempfile.TemporaryDirectory() as directory:
    root = Path(directory)
    data = root / 'data'
    data.mkdir()
    rgb = np.zeros((100, 100, 3), dtype=np.uint8)
    rgb[20:80, 30:70] = [255, 100, 40]
    Image.fromarray(rgb).save(data / 'input.png')
    mask = np.zeros((100, 100), dtype=np.uint8)
    mask[20:80, 30:70] = 255
    Image.fromarray(mask).save(data / 'mask.png')
    exclude = np.zeros_like(mask)
    exclude[20:40, 30:70] = 255
    Image.fromarray(exclude).save(data / 'hand.png')
    row = {'id': 'test', 'image': 'data/input.png', 'mask': 'data/mask.png', 'exclude_mask': 'data/hand.png'}
    result = np.array(runner.object_image(row, root))
    assert (result[:, :, 3] > 0).sum() == 1600, 'Hand exclusion failed'
    boxed = runner.object_image({'image': 'data/input.png', 'bbox': [25, 15, 75, 85]}, root)
    assert np.array(boxed)[:, :, 3].max() == 255
    Image.new('L', (100, 100)).save(data / 'empty.png')
    try:
        runner.object_image({**row, 'mask': 'data/empty.png'}, root)
    except ValueError:
        pass
    else:
        raise AssertionError('Empty mask was accepted')
    manifest = root / 'objects.json'
    manifest.write_text(json.dumps([row]), encoding='utf-8')
    subprocess.run([sys.executable, str(SCRIPT), 'reconstruct', '--manifest', str(manifest),
                    '--output', str(root / 'out'), '--prepare-only'], check=True)
    assert json.loads((root / 'out/test/result.json').read_text())['status'] == 'prepared'
    writer = cv2.VideoWriter(str(data / 'clip.avi'), cv2.VideoWriter_fourcc(*'MJPG'), 10, (100, 100))
    assert writer.isOpened()
    for _ in range(30):
        writer.write(rgb)
    writer.release()
    subprocess.run([sys.executable, str(SCRIPT), 'extract', '--input', str(data / 'clip.avi'),
                    '--output', str(root / 'frames'), '--max-frames', '2'], check=True)
    frames = json.loads((root / 'frames/frames.json').read_text())
    assert [f['frame'] for f in frames] == [0, 29]
print('PASS: object mask, hand exclusion, GrabCut, empty-mask rejection, CLI preview, video sampling')
