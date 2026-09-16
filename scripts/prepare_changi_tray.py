"""Prepare these burgundy tray photos; masks are colour-specific, not generic."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from PIL import Image, ImageOps, ImageDraw


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--input-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    dest = args.output
    (dest / 'images').mkdir(parents=True, exist_ok=True)
    preview = Image.new('RGB', (1200, 960), '#dddddd')
    rows = []
    for index in range(1, 5):
        with Image.open(args.input_dir / f'{index:02d}.jpg') as source:
            im = ImageOps.exif_transpose(source).convert('RGB')
        im.thumbnail((2048, 2048), Image.Resampling.LANCZOS)
        rgb = np.array(im)
        r, g, b = rgb.astype(np.float32).transpose(2, 0, 1)
        mask = ((r > 1.17*g) & (r > 1.10*b) & (r > 35)).astype('uint8')*255
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contour = max(contours, key=cv2.contourArea)
        mask[:] = 0
        # Fill the full outer silhouette, including the white printed graphics.
        cv2.drawContours(mask, [contour], -1, 255, cv2.FILLED)
        rgba = Image.fromarray(np.dstack([rgb, mask]))
        rgba.save(dest / 'images' / f'view{index:02d}.png')
        tile = Image.new('RGBA', rgba.size, '#dddddd')
        tile.alpha_composite(rgba)
        tile.thumbnail((590, 440), Image.Resampling.LANCZOS)
        x, y = ((index-1) % 2)*600, ((index-1)//2)*480
        preview.paste(tile.convert('RGB'), (x+(600-tile.width)//2, y+30))
        ImageDraw.Draw(preview).text((x+15, y+10), f'View {index:02d}', fill='black')
        rows.append({'id': f'tray_view{index:02d}', 'image': f'images/view{index:02d}.png'})
    for name, selection in [('primary', [rows[3]]), ('candidates', [rows[3], rows[1], rows[0]]), ('all', rows)]:
        (dest / f'objects_{name}.json').write_text(json.dumps(selection, indent=2), encoding='utf-8')
    preview.save(dest / 'mask_preview.jpg', quality=92)
    print(dest)


if __name__ == '__main__':
    main()
