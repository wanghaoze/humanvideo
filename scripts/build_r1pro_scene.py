"""Build a portable MJCF visualization scene from official URDF and exported objects."""
from pathlib import Path
import shutil
import json
import xml.etree.ElementTree as ET
import mujoco

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'outputs/r1pro_tray_scene'
SRC = ROOT / 'vendor/GalaxeaManipSim/galaxea_sim/assets/r1_pro'
robot_dir = OUT / 'assets/r1_pro'
robot_dir.mkdir(parents=True, exist_ok=True)
shutil.copytree(SRC / 'meshes', robot_dir / 'meshes', dirs_exist_ok=True)
for name in ['LICENSE', 'NOTICE']:
    shutil.copy(ROOT / 'vendor/GalaxeaManipSim' / name, robot_dir / name)
urdf = ET.parse(SRC / 'robot.urdf')
ext = ET.SubElement(urdf.getroot(), 'mujoco')
ET.SubElement(ext, 'compiler', discardvisual='false', fusestatic='false', balanceinertia='true')
for mesh in urdf.findall('.//mesh'):
    mesh.set('filename', str((robot_dir / mesh.get('filename')).resolve()))
urdf.write(robot_dir / 'converted.urdf')
model = mujoco.MjModel.from_xml_path(str(robot_dir / 'converted.urdf'))
mujoco.mj_saveLastXML(str(OUT / 'robot.xml'), model)
tree = ET.parse(OUT / 'robot.xml')
r = tree.getroot()
r.set('model', 'R1Pro_ChangiTray_Table')
compiler = r.find('compiler')
compiler.attrib.pop('meshdir', None)
for mesh in r.findall('./asset/mesh'):
    mesh.set('file', 'assets/r1_pro/meshes/' + Path(mesh.get('file').replace('\\', '/')).name)
asset = r.find('asset')
world = r.find('worldbody')
dimensions=json.loads((OUT/'object_dimensions.json').read_text())
# URDF visuals use group 1, collisions group 0. Keep the latter hidden by group 3.
for i, geom in enumerate(world.findall('.//geom')):
    geom.set('name', f'robot_geom_{i}')
    if geom.get('contype') == '0':
        geom.set('group', '1')
        geom.set('rgba', '0.72 0.75 0.78 1')
    else:
        geom.set('group', '3')
ET.SubElement(r, 'visual')
ET.SubElement(r.find('visual'), 'global', offwidth='1600', offheight='1000')
ET.SubElement(r.find('visual'), 'headlight', ambient='0.35 0.35 0.35', diffuse='0.7 0.7 0.7')
ET.SubElement(r, 'statistic', center='0.4 0 0.85', extent='2.5')
ET.SubElement(world, 'light', pos='0 -2 3', dir='0 0 -1', diffuse='0.8 0.8 0.8')
ET.SubElement(world, 'geom', name='floor', type='plane', size='4 4 0.05', rgba='0.19 0.22 0.27 1')
for key, pos, rgba in [('table','0.85 0 0','0.65 0.68 0.72 1'),('tray','0.85 0 0.801','0.27 0.025 0.045 1')]:
    body=ET.SubElement(world,'body',name=key,pos=pos)
    if key == 'table':
        body.set('quat', '0.7071067812 0 0 0.7071067812')
    for part in dimensions[key]['parts']:
        name=part['name']
        ET.SubElement(asset,'mesh',name=name,file=f'assets/{key}/{name}.obj')
        ET.SubElement(body,'geom',name=name+'_visual',type='mesh',mesh=name,rgba=' '.join(map(str,part['rgba'])),contype='0',conaffinity='0',group='1')
    if key=='table':
        ET.SubElement(body,'geom',name='table_top_collision',type='box',pos='0 0 0.78',size='0.6 0.3 0.02',group='3')
        for x in [-0.54,0.54]:
            for y in [-0.24,0.24]:
                ET.SubElement(body,'geom',type='box',pos=f'{x} {y} 0.38',size='0.025 0.025 0.38',group='3')
    else:
        ET.SubElement(body,'geom',name='tray_bottom_collision',type='box',pos='0 0 0.006',size='0.255 0.19 0.006',group='3')
ET.indent(tree)
tree.write(OUT / 'scene.xml')
model=mujoco.MjModel.from_xml_path(str(OUT / 'scene.xml'))
data=mujoco.MjData(model)
mujoco.mj_forward(model,data)
print('MODEL',model.nq,model.nbody,model.ngeom)
print('JOINTS', [(model.joint(i).name, model.jnt_range[i].tolist()) for i in range(model.njnt)])
(OUT/'validation.json').write_text(json.dumps({'nq':model.nq,'nbody':model.nbody,'ngeom':model.ngeom,'mujoco':mujoco.__version__},indent=2))
