"""Conda-only background lifecycle for the airport simulation collection service."""
import argparse,json,os,signal,socket,subprocess,sys,time,urllib.request
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from r1pro_teleop.airport_pipeline import save

def main():
 p=argparse.ArgumentParser();p.add_argument('command',choices=['start','stop','status']);p.add_argument('--root',type=Path,default=Path('runs/airport_collection_20260916'));p.add_argument('--scene-id',default='airport_0001_20260917');p.add_argument('--port',type=int,default=8767);a=p.parse_args()
 project=Path(__file__).resolve().parents[1];root=a.root.resolve();record_path=root/'agent/server_process.json';record=json.loads(record_path.read_text()) if record_path.exists() else {}
 def alive():
  try:return b'r1pro_teleop.airport_service' in Path(f"/proc/{record['pid']}/cmdline").read_bytes() and Path(f"/proc/{record['pid']}/cwd").resolve()==project
  except (KeyError,OSError):return False
 def health():
  if not alive():return {'running':False}
  try:
   token=(project/'runs/server.token').read_text().strip()
   with urllib.request.urlopen(urllib.request.Request(f"http://127.0.0.1:{record['port']}/api/state",headers={'Authorization':'Bearer '+token}),timeout=2) as r:s=json.load(r)
   return {'running':True,'ready':s['ready'],'scene_id':s.get('scene_id'),'paused':s.get('paused'),'recording':s.get('recording'),'base_control_available':False,'error':s.get('error') or s.get('render_error')}
  except Exception as e:return {'running':True,'ready':False,'error':str(e)}
 if a.command=='status':print(json.dumps({**record,**health()},indent=2));return
 if a.command=='stop':
  if alive():os.kill(record['pid'],signal.SIGTERM)
  for _ in range(100):
   if not alive():break
   time.sleep(.1)
  print(json.dumps(health()));return
 if not os.environ.get('CONDA_PREFIX') or Path(os.environ['CONDA_PREFIX']).resolve()!=Path(sys.prefix).resolve():raise RuntimeError('Use the existing r1pro-tray Conda environment')
 if alive():print(json.dumps({**record,**health()},indent=2));return
 with socket.socket() as s:s.bind(('0.0.0.0',a.port))
 from select_r1pro_gpu import select
 gpu=select(0);env=dict(os.environ,MUJOCO_GL='egl',MUJOCO_EGL_DEVICE_ID=str(gpu['egl_index']))
 logpath=root/'agent/server.log';logpath.parent.mkdir(exist_ok=True)
 with logpath.open('ab') as log:
  proc=subprocess.Popen([sys.executable,'-m','r1pro_teleop.airport_service','--root',str(root),'--scene-id',a.scene_id,'--port',str(a.port)],cwd=project,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
 record={'pid':proc.pid,'port':a.port,'root':str(root),'log':str(logpath)};save(record_path,record)
 for _ in range(100):
  status=health()
  if status.get('ready'):break
  time.sleep(.2)
 print(json.dumps({**record,**status},indent=2))
 if not status.get('ready'):raise RuntimeError('Airport service failed; inspect log')
if __name__=='__main__':main()
