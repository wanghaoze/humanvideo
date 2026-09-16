"""Re-run generated joint targets through physics and render an honest replay."""
import argparse,json,sys,hashlib
from pathlib import Path
import numpy as np,mujoco,av
from PIL import Image,ImageDraw
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from r1pro_teleop.core import Simulation
p=argparse.ArgumentParser();p.add_argument('directory',type=Path);p.add_argument('--scene',type=Path,default=Path('outputs/r1pro_tray_scene/scene_physics.xml'));a=p.parse_args()
metadata=json.loads((a.directory/'report.json').read_text());assert hashlib.sha256(a.scene.read_bytes()).hexdigest()==metadata['scene_sha256'],'Use the matching solver scene from report.json'
data=np.load(a.directory/'trajectory.npz');sim=Simulation(a.scene,width=640,height=480)
container=av.open(str(a.directory/'replay.mp4'),'w');stream=container.add_stream('libx264',rate=5);stream.width=640;stream.height=480;stream.pix_fmt='yuv420p';errors=[]
try:
 sim.data.qpos[:]=data['qpos'][0];sim.data.qvel[:]=data['qvel'][0];sim.data.qacc_warmstart[:]=data['warm'][0];mujoco.mj_forward(sim.model,sim.data)
 for i,action in enumerate(data['action']):
  errors.append(float(np.max(np.abs(sim.data.qpos-data['qpos'][i]))))
  if i%4==0:
   picture=Image.fromarray(sim.image('front'));draw=ImageDraw.Draw(picture);draw.rectangle((0,0,640,44),fill=(15,25,35));draw.text((10,6),'PROGRAM-GENERATED SIMULATION | '+str(data['stage'][i]),fill='white');draw.text((10,24),f"t={data['time'][i]:.1f}s  tray lift={(sim.data.body('tray').xpos[2]-.8)*100:.1f} cm",fill='white')
   frame=av.VideoFrame.from_ndarray(np.array(picture),format='rgb24')
   for packet in stream.encode(frame):container.mux(packet)
  sim.target[:]=action;sim.step()
 for packet in stream.encode():container.mux(packet)
finally:container.close();sim.close()
report={'max_qpos_replay_error':max(errors),'passed':max(errors)<1e-5,'frames':len(errors)};(a.directory/'replay_validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
