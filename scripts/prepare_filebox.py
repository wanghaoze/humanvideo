#!/usr/bin/env python3
"""Reproduce inspected frames and approximate visible-object masks for this episode."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw

from humanvideo_trellis2 import object_image, save_json

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DATA = '/home/michelle/haoze/EgoDemo/EgoProStandard-body/lerobot/Return File Box to Place'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-root', type=Path, default=Path(DEFAULT_DATA))
    parser.add_argument('--output', type=Path, default=ROOT / 'runs/filebox')
    parser.add_argument('--selection', type=Path, default=ROOT / 'examples/filebox_selection.json')
    args = parser.parse_args()
    selection = json.loads(args.selection.read_text(encoding='utf-8'))
    episode = args.data_root.resolve()
    if episode.name != selection['episode']:
        episode = episode / selection['episode']
    video = episode / f"videos/observation.images.{selection['camera']}/chunk-000/file-000.mp4"
    if not video.is_file():
        raise FileNotFoundError(video)
    out = args.output.resolve()
    if out == episode or episode in out.parents:
        raise ValueError('Write experiment outputs outside the source episode.')
    if out.exists() and any(out.glob('outputs*/**/object.glb')):
        raise FileExistsError('Existing GLB results: use a new --output to preserve their input provenance.')
    out.mkdir(parents=True, exist_ok=True)
    cap = cv2.VideoCapture(str(video))
    frames = {}
    try:
        fps = cap.get(cv2.CAP_PROP_FPS)
        count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        if not cap.isOpened() or count != selection['expected_frames'] or not np.isclose(fps, 30.006, atol=0.01):
            raise ValueError(f'Unexpected video: {count} frames, {fps} fps; masks are episode-specific.')
        for index in sorted({row['frame'] for row in selection['objects']}):
            cap.set(cv2.CAP_PROP_POS_FRAMES, index)
            ok, frame = cap.read()
            if not ok:
                raise ValueError(f'Cannot decode frame {index}')
            if list(frame.shape[1::-1]) != selection['expected_video_size']:
                raise ValueError('Video dimensions differ from the inspected source; do not reuse these polygons.')
            frames[index] = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    finally:
        cap.release()
    manifest = []
    sheet = Image.new('RGB', (480 * len(selection['objects']), 520), '#dddddd')
    for i, row in enumerate(selection['objects']):
        source = frames[row['frame']]
        image_name = f"frames/{selection['camera']}_{row['frame']:06d}.png"
        (out / 'frames').mkdir(exist_ok=True)
        (out / 'masks').mkdir(exist_ok=True)
        (out / 'previews').mkdir(exist_ok=True)
        source.save(out / image_name)
        rw, rh = selection['reference_size']
        polygon = [(round(x * source.width / rw), round(y * source.height / rh)) for x, y in row['polygon']]
        mask = Image.new('L', source.size)
        ImageDraw.Draw(mask).polygon(polygon, fill=255)
        mask_name = f"masks/{row['id']}.png"
        mask.save(out / mask_name)
        overlay = source.copy()
        ImageDraw.Draw(overlay).line(polygon + [polygon[0]], fill='red', width=5)
        overlay.save(out / f"previews/{row['id']}_outline.jpg")
        item = {'id': row['id'], 'image': image_name, 'mask': mask_name,
                'source_frame': row['frame'], 'source_seconds': row['frame'] / fps,
                'description': row['description'], 'mask_method': 'manually specified coarse polygon'}
        manifest.append(item)
        cutout = object_image(item, out)
        cutout.save(out / f"previews/{row['id']}.png")
        cutout.thumbnail((450, 450))
        sheet.paste(cutout, (i * 480 + (480-cutout.width)//2, 45), cutout)
        ImageDraw.Draw(sheet).text((i*480+12, 10), row['id'], fill='black')
    sheet.save(out / 'previews/contact_sheet.jpg')
    save_json(out / 'objects.json', manifest)
    save_json(out / 'objects_primary.json', manifest[:1])
    digest = hashlib.sha256()
    with video.open('rb') as file:
        for block in iter(lambda: file.read(1024 * 1024), b''):
            digest.update(block)
    save_json(out / 'source.json', {'video': str(video), 'video_sha256': digest.hexdigest(),
                                  'fps': fps, 'frame_count': count, 'selection': selection})
    print(f'Prepared {len(manifest)} candidates at {out}')
    print(f"Primary manifest: {out / 'objects_primary.json'}")
    print(f"Inspect: {out / 'previews/contact_sheet.jpg'}")


if __name__ == '__main__':
    main()
