"""Simulation-only actuated planar chassis; never a real robot base driver."""
from pathlib import Path
import xml.etree.ElementTree as ET
import hashlib, json, os
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation

JOINTS=('agent_base_x','agent_base_y','agent_base_yaw')

def make_mobile_scene(source,destination):
    source=Path(source).resolve();destination=Path(destination).resolve()
    if destination.exists():raise FileExistsError(destination)
    tree=ET.parse(source);root=tree.getroot();base=root.find(".//body[@name='base_link']")
    model=mujoco.MjModel.from_xml_path(str(source));data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    initial_rotation=data.body('base_link').xmat.reshape(3,3).copy()
    for i,(name,axis) in enumerate(zip(JOINTS,([1,0,0],[0,1,0],[0,0,1]))):
        axis=initial_rotation.T@axis
        base.insert(i,ET.Element('joint',name=name,type='hinge' if i==2 else 'slide',axis=' '.join(map(str,axis)),limited='false',damping='100' if i<2 else '50',armature='1'))
    actuators=root.find('actuator')
    for i,name in enumerate(JOINTS):
        ET.SubElement(actuators,'position',name=name+'_target',joint=name,kp='6000' if i<2 else '3000',kv='600' if i<2 else '300',ctrllimited='false',forcelimited='true',forcerange='-600 600' if i<2 else '-300 300')
    for key in root.findall('./keyframe/key'):
        if key.get('qpos'):key.set('qpos','0 0 0 '+key.get('qpos'))
        if key.get('qvel'):key.set('qvel','0 0 0 '+key.get('qvel'))
        if key.get('ctrl'):key.set('ctrl',key.get('ctrl')+' 0 0 0')
    # Carry the head view with the chassis, preserving its initial world extrinsics.
    head=root.find("./worldbody/camera[@name='head']")
    if head is not None:
        cid=model.camera('head').id
        position=initial_rotation.T@(data.cam_xpos[cid]-data.body('base_link').xpos)
        rotation=initial_rotation.T@data.cam_xmat[cid].reshape(3,3)
        for k in ('xyaxes','zaxis','euler','axisangle','quat'):head.attrib.pop(k,None)
        head.set('pos',' '.join(map(str,position)))
        head.set('quat',' '.join(map(str,Rotation.from_matrix(rotation).as_quat()[[3,0,1,2]])))
        root.find('worldbody').remove(head);base.append(head)
    for node in root.iter():
        if 'file' in node.attrib:node.set('file',os.path.relpath((source.parent/node.get('file')).resolve(),destination.parent).replace(chr(92),'/'))
    destination.parent.mkdir(parents=True,exist_ok=True);tree.write(destination,encoding='unicode')
    return hashlib.sha256(destination.read_bytes()).hexdigest()


def enable_collection(root):
    root=Path(root)
    if json.loads((root/'episodes/index.json').read_text()):raise ValueError('Use a fresh collection for a mobile profile')
    for row in json.loads((root/'scenes/index.json').read_text()):
        directory=root/'scenes'/row['scene_id'];spec=json.loads((directory/'scene.json').read_text())
        source=directory/'scene.xml';dest=directory/'mobile.xml'
        digest=make_mobile_scene(source,dest)
        spec.update(fixed_scene_xml_sha256=spec['xml_sha256'],xml_sha256=digest,scene_file='mobile.xml',base_control_available=True,base_model='simulation_ideal_planar_position_servo_v1',real_base_driver_available=False)
        spec['robot']['base_control_available']=True
        (directory/'scene.json').write_text(json.dumps(spec,indent=2))


def collision_pairs(sim):
    result=[]
    for i,c in enumerate(sim.data.contact):
        bodies=[int(sim.model.geom_bodyid[g]) for g in (c.geom1,c.geom2)]
        names=[sim.model.body(b).name for b in bodies]
        force=np.zeros(6);mujoco.mj_contactForce(sim.model,sim.data,i,force)
        if force[0]<.5:continue
        # Chassis wheels are expected to touch the floor.
        if 0 in bodies and any(n.startswith(('wheel_','steer_','base_link')) for n in names):continue
        robot=lambda n:n.startswith(('left_','right_','torso','base_','wheel_','steer_','zed_'))
        if any(robot(n) for n in names) and not any('gripper_finger' in n for n in names):
            result.append(dict(bodies=names,penetration_m=max(0.,float(-c.dist)),force_n=float(force[0])))
    return result


def move_base(sim,goal_world,emit=None,speed=.15,yaw_speed=.3):
    """Continuous servo motion, collision checked every 50 ms; no qpos teleport."""
    if not sim.base_control_available:raise ValueError('Scene has no actuated base')
    goal=np.asarray(goal_world,float)
    if goal.shape!=(3,) or not np.isfinite(goal).all():raise ValueError('Invalid base goal')
    start=sim.base_state().astype(float)
    delta=goal-start;delta[2]=np.arctan2(np.sin(delta[2]),np.cos(delta[2]))
    frames=max(20,int(np.ceil(max(np.linalg.norm(delta[:2])/speed,abs(delta[2])/yaw_speed)*sim.fps*1.5)))
    for frame in range(frames+20):
        u=min(1.,(frame+1)/frames);u=u*u*(3-2*u)
        sim.set_base_world_target(start+delta*u)
        if emit:emit('navigate_to_tray')
        sim.step()
        collisions=collision_pairs(sim)
        if collisions:
            sim.base_target=sim.base_joint_state().copy()
            raise RuntimeError('base_collision_guard: '+json.dumps(collisions[:3]))
    error=goal-sim.base_state();error[2]=np.arctan2(np.sin(error[2]),np.cos(error[2]))
    if np.linalg.norm(error[:2])>.015 or abs(error[2])>.03:raise RuntimeError('base_tracking_error: '+str(error))
    return dict(frames=frames+20,position_error_m=float(np.linalg.norm(error[:2])),yaw_error_rad=float(abs(error[2])))
