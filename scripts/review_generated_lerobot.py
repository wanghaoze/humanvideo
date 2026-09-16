"""Replay the exported episode's raw actions and review actual wrist RGB beside physics."""
import argparse,json,sys,hashlib
from pathlib import Path
import numpy as np,mujoco,h5py,av
from PIL import Image,ImageDraw
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from r1pro_teleop.core import Simulation
p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--scene',type=Path,required=True);a=p.parse_args()
raws=list((a.run/'raw').glob('*.h5'));assert len(raws)==1
s=Simulation(a.scene,width=1280,height=720);container=av.open(str(a.run/'review.mp4'),'w');stream=container.add_stream('libx264',rate=5);stream.width=1280;stream.height=720;stream.options={'crf':'18'};stream.pix_fmt='yuv420p';errors=[];directions={'left':[],'right':[]};ahead={'left':[],'right':[]};unflipped=[]
try:
 with h5py.File(raws[0],'r') as f:
  assert f.attrs['scene_sha256']==hashlib.sha256(a.scene.read_bytes()).hexdigest()
  s.data.qpos[:]=f['qpos'][0];s.data.qvel[:]=f['qvel'][0];s.data.qacc_warmstart[:]=f['qacc_warmstart'][0];mujoco.mj_forward(s.model,s.data)
  for i,action in enumerate(f['action']):
   errors.append(float(np.max(abs(s.data.qpos-f['qpos'][i]))))
   unflipped.extend(float(s.tcp(side)[1][1,1]) for side in directions)
   if f['contact/left_force'][i]>0 and f['contact/right_force'][i]>0:
    for side in directions:
     directions[side].append(float(-s.data.camera(side+'_wrist').xmat.reshape(3,3)[0,2]))
     ahead[side].append(float(s.data.body(side+'_realsense_link').xpos[0]-s.tcp(side)[0][0]))
   if i%4==0:
    picture=Image.new('RGB',(1280,720))
    panels=[('FRONT',s.image('front'),(0,0)),('HEAD',f['images/head'][i],(640,0)),('LEFT WRIST',f['images/left_wrist'][i],(0,360)),('RIGHT WRIST',f['images/right_wrist'][i],(640,360))]
    for name,rgb,(x,y) in panels:
     picture.paste(Image.fromarray(rgb).resize((640,360),Image.Resampling.LANCZOS),(x,y))
     draw=ImageDraw.Draw(picture);draw.rectangle((x,y,x+640,y+24),fill=(15,25,35));draw.text((x+8,y+5),f'{name} | SIM PROGRAM GENERATED | t={i/20:.1f}s',fill='white')
    frame=av.VideoFrame.from_ndarray(np.array(picture),format='rgb24')
    for packet in stream.encode(frame):container.mux(packet)
   s.target[:]=action;s.step()
  for packet in stream.encode():container.mux(packet)
finally:container.close();s.close()
report={'physical_camera_ahead_min_m':{side:min(v) for side,v in ahead.items()},'direct_grasp_passed':all(v and min(v)>0 for v in ahead.values()) and min(unflipped)>0,'wrist_unflipped_min':min(unflipped),'max_qpos_replay_error':max(errors),'replay_passed':max(errors)<1e-5,'frames':len(errors),'grasp_outward_direction_min':{side:min(v) for side,v in directions.items()},'both_cameras_outward_during_dual_contact':all(min(v)>0 for v in directions.values())};(a.run/'review_validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
