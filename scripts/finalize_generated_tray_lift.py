"""Validate and export a generated lift against its explicitly named scene."""
import sys,json,hashlib,argparse
from pathlib import Path
import numpy as np,mujoco
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from r1pro_teleop.core import Simulation,NAMES
p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--scene',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
source=np.load(a.input);actions=np.r_[source['action'],np.repeat(source['action'][-1:],100,axis=0)];stages=list(source['stage'])+['hold']*100
s=Simulation(a.scene,render=False);rows=[];contacts=[]
try:
 for action,stage in zip(actions,stages):
  s.target=action.copy();forces={'left':0.,'right':0.};table=0;bad=[]
  for n,c in enumerate(s.data.contact):
   names=[s.model.body(int(s.model.geom_bodyid[g])).name for g in [c.geom1,c.geom2]]
   if 'tray' in names:
    f=np.zeros(6);mujoco.mj_contactForce(s.model,s.data,n,f)
    for side in forces:
     if any(v.startswith(side+'_gripper_finger') for v in names):forces[side]+=float(f[0])
    table+=int('table' in names)
   elif any(v.startswith(('left_','right_')) for v in names):bad.append(names)
  body=s.data.body('tray');tilt=float(np.degrees(np.arccos(np.clip(body.xmat.reshape(3,3)[2,2],-1,1))))
  verts=[]
  for g in range(s.model.ngeom):
   if s.model.geom_bodyid[g]!=s.model.body('tray').id or s.model.geom_contype[g]==0:continue
   mesh=int(s.model.geom_dataid[g]);start=int(s.model.mesh_vertadr[mesh]);count=int(s.model.mesh_vertnum[mesh]);v=s.model.mesh_vert[start:start+count]
   verts.append(float(np.min((v@s.data.geom_xmat[g].reshape(3,3).T+s.data.geom_xpos[g])[:,2])))
  camera_front={side:float(s.data.body(side+'_realsense_link').xpos[0]-s.tcp(side)[0][0]) for side in ('left','right')}
  wrist_unflipped={side:float(s.tcp(side)[1][1,1]) for side in ('left','right')}
  contacts.append(dict(camera_front=camera_front,wrist_unflipped=wrist_unflipped,stage=stage,forces=forces,table_contacts=table,bad_contacts=bad,tilt_deg=tilt,clearance_m=min(verts)-.8))
  rows.append(dict(time=float(s.data.time),qpos=s.data.qpos.copy(),qvel=s.data.qvel.copy(),warm=s.data.qacc_warmstart.copy(),state=s.state().copy(),action=s.target.copy(),tray=body.xpos.copy()))
  s.step()
 held=[i for i,v in enumerate(stages) if v=='hold'];zs=np.array([rows[i]['tray'][2] for i in held]);valid=[contacts[i] for i in held]
 report=dict(source='program_generated_simulation',human_demonstration=False,real_robot_execution=False,scene=str(a.scene),scene_sha256=hashlib.sha256(a.scene.read_bytes()).hexdigest(),fps=20,frames=len(rows),duration_s=len(rows)/20,hold_seconds=len(held)/20,hold_min_height_above_table_m=float(min(zs)-.8),hold_vertical_drift_m=float(np.ptp(zs)),hold_min_mesh_clearance_m=min(v['clearance_m'] for v in valid),max_hold_tilt_deg=max(v['tilt_deg'] for v in valid),hold_contact_both_hands=all(all(f>0 for f in v['forces'].values()) for v in valid),hold_table_contacts=sum(v['table_contacts'] for v in valid),bad_contact_frames=sum(bool(v['bad_contacts']) for v in contacts),parameters={'tray_mass_kg':1.2,'finger_sliding_friction':1.2,'gripper_kp':2000,'gripper_kv':40,'gripper_force_limit_N':40,'solver_impratio':10,'baseline_impratio':1},names=NAMES,action_semantics='absolute joint position targets rad, summed finger opening m',observation_semantics='state at t before action[t]')
 report['camera_front_min_m']={side:min(v['camera_front'][side] for v in contacts if v['stage'] in ('descend','close','lift','hold')) for side in ('left','right')}
 report['wrist_unflipped_min']={side:min(v['wrist_unflipped'][side] for v in contacts) for side in ('left','right')}
 report['direct_grasp_passed']=all(v>0 for v in report['camera_front_min_m'].values()) and all(v>0 for v in report['wrist_unflipped_min'].values())
 report['success']=report['direct_grasp_passed'] and report['hold_min_mesh_clearance_m']>.1 and report['hold_vertical_drift_m']<.01 and report['hold_contact_both_hands'] and report['hold_table_contacts']==0 and report['bad_contact_frames']==0
 np.savez_compressed(a.output/'trajectory.npz',**{k:np.array([r[k] for r in rows]) for k in rows[0]},stage=np.array(stages))
 np.savetxt(a.output/'actions.csv',np.c_[np.array([r['time'] for r in rows]),actions],delimiter=',',header='time_s,'+','.join(NAMES),comments='')
 (a.output/'report.json').write_text(json.dumps(report,indent=2));(a.output/'contacts.json').write_text(json.dumps(contacts));print(json.dumps(report,indent=2))
finally:s.close()
