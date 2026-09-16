"""Read literal R1 Pro initial joint arrays without executing a ROS launch file."""
import ast,argparse,json,hashlib,sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
p=argparse.ArgumentParser();p.add_argument('--launch',type=Path,required=True);p.add_argument('--base-scene',type=Path,default=Path('outputs/r1pro_tray_scene/scene_physics.xml'));p.add_argument('--output',type=Path,default=Path('outputs/r1pro_tray_scene/scene_launch_pose.xml'));a=p.parse_args()
keys=['vr_initializer_'+part+'_target_joint_states_r1pro' for part in ['left','right','torso']];values={}
for node in ast.walk(ast.parse(a.launch.read_text(encoding='utf-8'))):
 if isinstance(node,ast.Dict):
  for k,v in zip(node.keys,node.values):
   if isinstance(k,ast.Constant) and k.value in keys:values[k.value]=ast.literal_eval(v)
assert all(k in values for k in keys)
tree=ET.parse(a.base_scene);root=tree.getroot();source=ET.parse(a.base_scene.with_name('scene.xml')).getroot()
for i,q in enumerate(values[keys[2]],1):
 original=source.find(f'.//body[@name="torso_link{i}"]');joint=original.find('joint');axis=np.fromstring(joint.get('axis','0 0 1'),sep=' ')
 body=root.find(f'.//body[@name="torso_link{i}"]');quat=Rotation.from_rotvec(axis*q).as_quat();body.set('quat',' '.join(map(str,quat[[3,0,1,2]])))
root.find('option').set('impratio','10')
for side in ['left','right']:
 root.find(f'.//camera[@name="{side}_wrist"]').set('xyaxes','0 1 0 -0.67267279 0 0.73994007')
head=root.find('.//camera[@name="head"]');pos=np.array([.68,0,1.60]);target=np.array([.95,0,.86]);z=pos-target;z/=np.linalg.norm(z);x=np.cross([0,0,1],z);x/=np.linalg.norm(x);y=np.cross(z,x);head.set('pos',' '.join(map(str,pos)));head.set('xyaxes',' '.join(map(str,np.r_[x,y])))

ET.indent(tree);tree.write(a.output,encoding='unicode')
model=mujoco.MjModel.from_xml_path(str(a.output.resolve()));qpos=model.qpos0.copy()
for side,key in zip(['left','right'],keys[:2]):
 assert len(values[key])==7
 for i,value in enumerate(values[key],1):
  j=model.joint(f'{side}_arm_joint{i}').id;assert model.jnt_range[j,0]<=value<=model.jnt_range[j,1];qpos[model.jnt_qposadr[j]]=value
 for i in [1,2]:qpos[model.jnt_qposadr[model.joint(f'{side}_gripper_finger_joint{i}').id]]=.05
keyframe=root.find('keyframe')
if keyframe is None:keyframe=ET.SubElement(root,'keyframe')
existing=keyframe.find('key[@name="teleop_home"]')
if existing is not None:keyframe.remove(existing)
ET.SubElement(keyframe,'key',name='teleop_home',qpos=' '.join(map(str,qpos)))
ET.indent(tree);tree.write(a.output,encoding='unicode')
report={'launch_source':str(a.launch),'launch_sha256':hashlib.sha256(a.launch.read_bytes()).hexdigest(),'joint_values':values,'torso_mode':'fixed at supplied joint angles; arms and grippers actuated','scene':str(a.output),'scene_sha256':hashlib.sha256(a.output.read_bytes()).hexdigest()};a.output.with_suffix('.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
