"""Re-execute synthetic commands, record synchronized RGB/state, export LeRobot v3."""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np,mujoco
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from r1pro_teleop.core import Simulation,Recorder,CAMERAS
from r1pro_teleop.dataset import export
from r1pro_teleop.quality import inspect

def main():
 p=argparse.ArgumentParser();p.add_argument('--trajectory',type=Path,required=True);p.add_argument('--scene',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--repo-id',default='local/r1pro-synthetic-tray-lift');p.add_argument('--max-step',type=float,default=.06);p.add_argument('--width',type=int,default=1280);p.add_argument('--height',type=int,default=720);a=p.parse_args()
 if a.output.exists():raise FileExistsError(a.output)
 if not 0<a.max_step<=.1:raise ValueError('max-step must be in (0,.1] rad')
 a.output.mkdir(parents=True)
 data=np.load(a.trajectory);s=Simulation(a.scene,width=a.width,height=a.height)
 # Initial pose must match the configured scene reset; do not teleport to a grasp.
 if not np.allclose(s.data.qpos,data['qpos'][0],atol=1e-5):raise ValueError('Trajectory initial pose does not match scene reset')
 actions=[]
 old=s.target.copy()
 for row in data['action']:
  steps=max(1,int(np.ceil(np.max(abs(row-old))/a.max_step)))
  actions.extend(old+(row-old)*k/steps for k in range(1,steps+1));old=row.copy()
 rec=Recorder(a.output/'raw',s,task='Lift the empty tray with both grippers and hold it above the table',controller_source='program_generated',task_kind='tray_lift')
 rec.file.attrs['generation_input_sha256']=hashlib.sha256(a.trajectory.read_bytes()).hexdigest()
 rec.file.attrs['retiming_max_joint_step_rad']=a.max_step
 rec.file.attrs['grasp_style']='direct_camera_front'
 preview_dir=a.output/'camera_previews';preview_dir.mkdir()
 try:
  for i,action in enumerate(actions):
   s.target=np.asarray(action,dtype=np.float32);forces={'left':0.,'right':0.};table=0;other=0
   for n,c in enumerate(s.data.contact):
    names=[s.model.body(int(s.model.geom_bodyid[g])).name for g in (c.geom1,c.geom2)]
    if 'tray' in names:
     f=np.zeros(6);mujoco.mj_contactForce(s.model,s.data,n,f)
     table+=int('table' in names)
     for side in forces:
      if any(name.startswith(side+'_gripper_finger') for name in names):forces[side]+=float(f[0])
    elif any(name.startswith(('left_','right_')) for name in names):other+=1
   extras={'contact/left_force':forces['left'],'contact/right_force':forces['right'],'contact/table_count':table,'contact/other_count':other}
   for side in forces:
    c=s.data.camera(side+'_wrist');direction=-c.xmat.reshape(3,3)[:,2]
    # Positive world X is from the fixed robot toward the tabletop.
    extras['camera_check/'+side+'_outward']=float(direction[0])
    extras['camera_check/'+side+'_ahead']=float(s.data.body(side+'_realsense_link').xpos[0]-s.tcp(side)[0][0])
    extras['camera_check/'+side+'_unflipped']=float(s.tcp(side)[1][1,1])
   images=s.images();rec.append(s,images,-1,extras)
   if i in (0,len(actions)//2,len(actions)-1):
    from PIL import Image
    for name,rgb in images.items():Image.fromarray(rgb).save(preview_dir/f'{i:04d}_{name}.jpg')
   s.step()
   if i%100==0:print(f'recorded {i}/{len(actions)}',flush=True)
  path=rec.finish('unlabeled')
 finally:s.close()
 quality=inspect(path)
 # A generated success is assigned only after all lift/contact/camera checks.
 if quality.get('valid') and quality['task']['candidate_success'] and not quality['warnings']:
  import h5py
  with h5py.File(path,'r+') as f:f.attrs['result']='success';f.attrs['success_operator']=False
  quality=inspect(path)
 (a.output/'quality.json').write_text(json.dumps(quality,indent=2))
 if not quality['training_candidate']:raise RuntimeError('Generated episode rejected; inspect quality.json')
 export(a.output/'raw',a.output/'lerobot',a.repo_id,require_task_checks=True)
 print('LeRobot dataset verified:',a.output/'lerobot',flush=True)

if __name__=='__main__':main()
