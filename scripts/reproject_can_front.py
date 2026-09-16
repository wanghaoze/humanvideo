"""Bake an orthographic front photograph into the existing GLB UVs.

For the inspected can: glTF Y is vertical, +Z is the selected front.
Preserves the original binary geometry, normals, UVs and material parameters.
Requires numpy, Pillow and opencv-python-headless; does not invoke TRELLIS.
"""
import argparse
import hashlib
import io
import json
from pathlib import Path
import struct

import cv2
import numpy as np
from PIL import Image


def read_glb(path):
    data = path.read_bytes()
    magic, version, size = struct.unpack_from('<III', data)
    if (magic, version, size) != (0x46546C67, 2, len(data)):
        raise ValueError('Invalid GLB')
    chunks = {}
    pos = 12
    while pos < len(data):
        size, kind = struct.unpack_from('<II', data, pos)
        pos += 8
        chunks[kind] = data[pos:pos+size]
        pos += size
    return json.loads(chunks[0x4E4F534A]), chunks[0x004E4942]


def accessor(doc, binary, index):
    a = doc['accessors'][index]
    view = doc['bufferViews'][a['bufferView']]
    components = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3}[a['type']]
    dtype = np.dtype({5126: '<f4', 5125: '<u4', 5123: '<u2'}[a['componentType']])
    return np.ndarray((a['count'], components), dtype=dtype, buffer=binary,
                      offset=view.get('byteOffset', 0)+a.get('byteOffset', 0),
                      strides=(view.get('byteStride', dtype.itemsize*components), dtype.itemsize)).copy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--glb', type=Path, required=True)
    parser.add_argument('--image', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--double-sided-label', action='store_true', help='Repeat the photo on the back and feather both labels into clean side colors.')
    args = parser.parse_args()
    if args.output.resolve() == args.glb.resolve():
        raise ValueError('Use a separate output file.')
    doc, binary = read_glb(args.glb)
    if len(doc['meshes']) != 1 or len(doc['meshes'][0]['primitives']) != 1:
        raise ValueError('This tool expects the inspected single-mesh can.')
    prim = doc['meshes'][0]['primitives'][0]
    verts = accessor(doc, binary, prim['attributes']['POSITION'])
    uv = accessor(doc, binary, prim['attributes']['TEXCOORD_0'])
    faces = accessor(doc, binary, prim['indices']).reshape(-1, 3)
    tex_index = doc['materials'][prim['material']]['pbrMetallicRoughness']['baseColorTexture']['index']
    tex = doc['textures'][tex_index]
    image_index = tex.get('source', tex.get('extensions', {}).get('EXT_texture_webp', {}).get('source'))
    image = doc['images'][image_index]
    view = doc['bufferViews'][image['bufferView']]
    offset = view.get('byteOffset', 0)
    old = np.array(Image.open(io.BytesIO(binary[offset:offset+view['byteLength']])).convert('RGBA'))
    result = old.copy()
    source = np.array(Image.open(args.image).convert('RGBA'))
    alpha = source[:, :, 3] > 127
    ys, xs = np.where(alpha)
    top, bottom = ys.min(), ys.max()
    left = np.full(source.shape[0], xs.min(), np.float32)
    right = np.full(source.shape[0], xs.max(), np.float32)
    for y in range(top, bottom+1):
        row = np.flatnonzero(alpha[y])
        if len(row):
            left[y], right[y] = row[0], row[-1]
    base_rows = np.zeros((source.shape[0], 3), np.float32)
    if args.double_sided_label:
        for y in range(top, bottom+1):
            colors = source[y, alpha[y], :3].astype(np.float32)
            if len(colors):
                red = (colors[:, 0] > 1.8*colors[:, 1]) & (colors[:, 0] > 1.8*colors[:, 2])
                base_rows[y] = np.median(colors[red] if red.sum() > 10 else colors, axis=0)
    lo, hi = verts.min(0), verts.max(0)
    center = (lo+hi)/2
    # Local horizontal silhouette per height bin, to align the photo's rim/shoulder.
    bins = 256
    yi = np.clip(((verts[:, 1]-lo[1])/(hi[1]-lo[1])*(bins-1)).astype(int), 0, bins-1)
    radii = np.zeros(bins, np.float32)
    np.maximum.at(radii, yi, np.abs(verts[:, 0]-center[0]))
    valid = radii > 0
    radii = np.interp(np.arange(bins), np.flatnonzero(valid), radii[valid])
    h, w = old.shape[:2]
    raster_uv = uv*np.array([w, h]) - 0.5
    coverage = np.zeros((h, w), np.uint8)
    changed = np.zeros((h, w), np.uint8)
    for fi, face in enumerate(faces):
        tri = raster_uv[face]
        xmin, ymin = np.maximum(np.ceil(tri.min(0)).astype(int), 0)
        xmax, ymax = np.minimum(np.floor(tri.max(0)).astype(int), [w-1, h-1])
        if xmax < xmin or ymax < ymin:
            continue
        a, b, c = tri
        den = (b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1])
        if abs(den) < 1e-10:
            continue
        gx, gy = np.meshgrid(np.arange(xmin, xmax+1), np.arange(ymin, ymax+1))
        wa = ((b[1]-c[1])*(gx-c[0])+(c[0]-b[0])*(gy-c[1]))/den
        wb = ((c[1]-a[1])*(gx-c[0])+(a[0]-c[0])*(gy-c[1]))/den
        wc = 1-wa-wb
        inside = (wa >= -1e-6) & (wb >= -1e-6) & (wc >= -1e-6)
        px, py = gx[inside], gy[inside]
        coverage[py, px] = 1
        if not len(px):
            continue
        xyz = wa[inside, None]*verts[face[0]] + wb[inside, None]*verts[face[1]] + wc[inside, None]*verts[face[2]]
        radial = xyz[:, [0, 2]] - center[[0, 2]]
        facing = radial[:, 1]/np.maximum(np.linalg.norm(radial, axis=1), 1e-8)
        # Full photo on the front, feather only the near-tangent band.
        weight = np.clip((facing-0.08)/0.25, 0, 1)
        weight = weight*weight*(3-2*weight)
        if args.double_sided_label:
            weight = np.clip((np.abs(facing)-0.15)/0.40, 0, 1)
            weight = weight*weight*(3-2*weight)
        keep = weight > 0
        if args.double_sided_label:
            keep = np.ones(len(weight), dtype=bool)
        xyz, px, py, weight = xyz[keep], px[keep], py[keep], weight[keep]
        if not len(px):
            continue
        normalized_y = np.clip((xyz[:, 1]-lo[1])/(hi[1]-lo[1]), 0, 1)
        sy = bottom-normalized_y*(bottom-top)
        radius = np.interp(normalized_y*(bins-1), np.arange(bins), radii)
        nx = np.clip((xyz[:, 0]-center[0])/np.maximum(radius, 1e-6), -1, 1)
        if args.double_sided_label:
            nx *= np.where(xyz[:, 2] >= center[2], 1, -1)
        sl = np.interp(sy, np.arange(len(left)), left)
        sr = np.interp(sy, np.arange(len(right)), right)
        sx = (sl+sr)/2 + nx*(sr-sl)/2
        sampled = cv2.remap(source, sx.astype(np.float32).reshape(1, -1), sy.astype(np.float32).reshape(1, -1),
                            cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE).reshape(-1, 4)
        weight *= sampled[:, 3]/255
        background = old[py, px, :3]
        if args.double_sided_label:
            background = np.stack([np.interp(sy, np.arange(len(base_rows)), base_rows[:, c]) for c in range(3)], axis=1)
        colors = weight[:, None]*sampled[:, :3] + (1-weight[:, None])*background
        result[py, px, :3] = np.round(colors).astype(np.uint8)
        changed[py[weight > 0], px[weight > 0]] = 1
        if fi % 20000 == 0:
            print(f'Baking triangle {fi}/{len(faces)}', flush=True)
    # Extend island colors into four gutter pixels to prevent filtering seams.
    filled = coverage.astype(np.float32)
    for _ in range(4):
        counts = cv2.blur(filled, (3, 3))
        border = (filled == 0) & (counts > 0)
        for channel in range(3):
            sums = cv2.blur(result[:, :, channel].astype(np.float32)*filled, (3, 3))
            result[:, :, channel][border] = np.clip(sums[border]/counts[border], 0, 255).astype(np.uint8)
        filled[border] = 1
    args.output.parent.mkdir(parents=True, exist_ok=True)
    texture_path = args.output.with_name(args.output.stem+'_basecolor.png')
    Image.fromarray(result).save(texture_path)
    png = texture_path.read_bytes()
    new_binary = binary + b'\0'*((-len(binary)) % 4)
    doc['bufferViews'].append({'buffer': 0, 'byteOffset': len(new_binary), 'byteLength': len(png)})
    doc['images'][image_index] = {'bufferView': len(doc['bufferViews'])-1, 'mimeType': 'image/png'}
    tex.pop('extensions', None)
    tex['source'] = image_index
    new_binary += png
    doc['buffers'][0]['byteLength'] = len(new_binary)
    encoded = json.dumps(doc, separators=(',', ':')).encode()
    encoded += b' '*((-len(encoded)) % 4)
    new_binary += b'\0'*((-len(new_binary)) % 4)
    total = 12+8+len(encoded)+8+len(new_binary)
    args.output.write_bytes(struct.pack('<III', 0x46546C67, 2, total) + struct.pack('<II', len(encoded), 0x4E4F534A)
                            + encoded + struct.pack('<II', len(new_binary), 0x004E4942) + new_binary)
    new_doc, new_bin = read_glb(args.output)
    assert new_bin[:len(binary)] == binary, 'Original binary payload changed'
    for key in ['POSITION', 'NORMAL', 'TEXCOORD_0']:
        ai = prim['attributes'][key]
        assert np.array_equal(accessor(doc, binary, ai), accessor(new_doc, new_bin, ai))
    assert np.array_equal(faces, accessor(new_doc, new_bin, prim['indices']).reshape(-1, 3))
    report = {'source_glb': str(args.glb), 'source_image': str(args.image), 'output_glb': str(args.output),
              'vertices': len(verts), 'triangles': len(faces), 'geometry_and_uv_unchanged': True,
              'front_axis': '+Z in glTF coordinates', 'vertical_axis': '+Y',
              'texture_size': [w, h], 'painted_texels': int(changed.sum()),
              'original_glb_sha256': hashlib.sha256(args.glb.read_bytes()).hexdigest(),
              'note': ('Two repeated photo labels with clean feathered sides; synthesized back, not a true back photograph.'
                       if args.double_sided_label else 'Front photo projection only; back remains generated. Original PBR parameters preserved.')}
    args.output.with_suffix('.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
