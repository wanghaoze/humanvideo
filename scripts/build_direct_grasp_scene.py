"""Preserve the supplied launch pose/physics; configure outward cameras and 720p."""
import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument('--source', type=Path, default=Path('outputs/r1pro_tray_scene/scene_launch_pose.xml'))
p.add_argument('--output', type=Path, default=Path('outputs/r1pro_tray_scene/scene_direct_grasp_720p.xml'))
a = p.parse_args()
tree = ET.parse(a.source)
root = tree.getroot()
visual = root.find('visual')
if visual is None:
    visual = ET.SubElement(root, 'visual')
global_view = visual.find('global')
if global_view is None:
    global_view = ET.SubElement(visual, 'global')
global_view.set('offwidth', '1280')
global_view.set('offheight', '720')
for side in ('left', 'right'):
    camera = root.find(f'.//camera[@name="{side}_wrist"]')
    camera.set('pos', '0.055 0.003 -0.01')
    camera.set('xyaxes', '0 -1 0 0.866025404 0 0.5')
    camera.set('fovy', '80')
ET.indent(tree)
tree.write(a.output, encoding='unicode')
