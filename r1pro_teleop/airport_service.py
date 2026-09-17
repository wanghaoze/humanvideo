"""Airport Quest service: authenticated existing protocol, explicit episode lifecycle."""
import argparse,io,json,os,time,threading,uuid
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import mujoco
import numpy as np
from PIL import Image,ImageDraw
import uvicorn
from .server import Service,create_app
from .airport_recording import AirportSimulation,AirportRecorder
from .airport_pipeline import load_scene,finish_episode,save

class AirportService(Service):
    viewer_file='airport_viewer.html'
    allowed_commands=('record','save_success','save_failure','save_unlabeled','abort','reset','reset_paused','pause','next_scene')
    def __init__(self,root,scene_id):
        self.root=Path(root).resolve();self.registry=json.loads((self.root/'scenes/index.json').read_text())
        self.scene_ids=[e['scene_id'] for e in self.registry];self.index=self.scene_ids.index(scene_id)
        self.spec,scene=load_scene(self.root,scene_id);super().__init__(scene,self.root/'episodes')
        self.generation=0;self.jobs=ThreadPoolExecutor(max_workers=1);self.futures=[]
        self.state.update(base_control_available=False,notice='SIMULATION ONLY | BASE UNAVAILABLE | Scene reset does not navigate',controller_source='human_teleoperation')
    def render_views(self):
        view=None;generation=-1
        try:
            while not self.stop.is_set():
                with self.lock:snapshot=self.snapshot;scene=self.scene;current=self.generation
                if snapshot is None:self.stop.wait(.02);continue
                if generation!=current:
                    if view:view.close()
                    view=AirportSimulation(scene,width=640,height=480);generation=current
                if snapshot[4]!=generation:continue
                view.data.qpos[:]=snapshot[0];view.data.qvel[:]=snapshot[1];view.data.time=snapshot[2];mujoco.mj_forward(view.model,view.data)
                images={c:Image.fromarray(view.image(c)) for c in ('overview','front','left_wrist','right_wrist')};grid=Image.new('RGB',(640,480))
                for i,(name,im) in enumerate(images.items()):grid.paste(im.resize((320,240)),((i%2)*320,(i//2)*240))
                images['grid']=grid;encoded={}
                for name,im in images.items():
                    draw=ImageDraw.Draw(im);draw.rectangle((0,0,640,20),fill='#243344');draw.text((6,4),'SIMULATION | BASE UNAVAILABLE | '+self.spec['scene_id'],fill='white')
                    buf=io.BytesIO();im.save(buf,format='JPEG',quality=88);encoded[name]=buf.getvalue()
                with self.lock:
                    if generation==self.generation:self.views=encoded;self.state['last_frame_monotonic']=snapshot[3]
                self.stop.wait(.08)
        except Exception as e:
            with self.lock:self.state['render_error']=str(e)
        finally:
            if view:view.close()
    def finalized(self,future):
        try:
            result=future.result()
            with self.lock:self.state['last_completed_episode']=result
        except Exception as e:
            with self.lock:self.state['pipeline_error']=str(e)
    def submit(self,path):
        future=self.jobs.submit(finish_episode,self.root,path);future.add_done_callback(self.finalized);self.futures.append(future)
    def initial_matches(self,sim):
        return all(np.allclose(sim.data.body(t['tray_id']).xpos,t['position_world'],atol=1e-8) and np.allclose(sim.data.body(t['tray_id']).xquat,[np.cos(t['yaw_world']/2),0,0,np.sin(t['yaw_world']/2)],atol=1e-8) for t in self.spec['trays'])
    def run(self):
        sim=None;rec=None;handled=None;previous={};renderer=threading.Thread(target=self.render_views,daemon=True);renderer.start()
        try:
            sim=AirportSimulation(self.scene,width=1280,height=720,render=False);sim.paused=True
            self.state['initial_scene_matches_json']=self.initial_matches(sim)
            while not self.stop.is_set():
                begin=time.monotonic()
                with self.lock:incoming=self.incoming
                if incoming and incoming['id']!=handled:
                    sim.input(incoming['packet'],now=incoming['received']);handled=incoming['id'];buttons=incoming['packet'].get('buttons',{})
                    if begin-incoming['received']<.25:
                        for key,cmd in [('X','record'),('A','save_success'),('B','pause')]:
                            if buttons.get(key) and not previous.get(key):self.commands.put_nowait(cmd)
                    previous=buttons
                while not self.commands.empty():
                    cmd=self.commands.get_nowait()
                    if cmd=='record' and rec is None:
                        if not sim.last_packet_time or begin-sim.last_packet_time>.25:
                            self.state['command_error']='Fresh Quest input required for a human episode';continue
                        directory=self.root/'episodes'/('capture_'+uuid.uuid4().hex);rec=AirportRecorder(directory,sim,self.spec,'human_teleoperation');sim.paused=False
                        if sim.renderer is None:sim.renderer=mujoco.Renderer(sim.model,height=720,width=1280)
                    elif cmd in ('save_success','save_failure','save_unlabeled','abort') and rec:
                        path=rec.finish('aborted' if cmd=='abort' else cmd.removeprefix('save_'));rec=None;sim.paused=True;self.submit(path)
                    elif cmd in ('reset','reset_paused','next_scene'):
                        if rec:path=rec.finish('aborted');rec=None;self.submit(path)
                        if cmd=='next_scene':
                            self.index=(self.index+1)%len(self.scene_ids);spec,scene=load_scene(self.root,self.scene_ids[self.index])
                            sim.close();sim=AirportSimulation(scene,width=1280,height=720,render=False)
                            with self.lock:self.spec=spec;self.scene=scene;self.generation+=1;self.snapshot=None;self.views={}
                        sim.reset();sim.paused=True;handled=None;previous={}
                        self.state['initial_scene_matches_json']=self.initial_matches(sim)
                        with self.lock:self.incoming=None;self.owner=None;self.sequence=-1
                    elif cmd=='pause':sim.paused=not sim.paused;sim.anchors.clear()
                sim.update_control()
                if rec:rec.append(sim,sim.images(),begin-sim.last_packet_time if sim.last_packet_time else -1)
                with self.lock:
                    self.snapshot=(sim.data.qpos.copy(),sim.data.qvel.copy(),float(sim.data.time),time.monotonic(),self.generation)
                    self.state.update(ready=True,scene_id=self.spec['scene_id'],split=self.spec['split'],recording=rec is not None,frames=rec.count if rec else 0,paused=sim.paused,state=sim.state().tolist(),action=sim.target.tolist(),tray_xyz=[sim.data.body(t['tray_id']).xpos.tolist() for t in self.spec['trays']],wrist_guard_active=sim.wrist_guard_active,pending_exports=sum(not f.done() for f in self.futures),output_root=str(self.root))
                sim.step();self.state['processing_ms']=(time.monotonic()-begin)*1000;self.stop.wait(max(0,.05-(time.monotonic()-begin)))
        except Exception as e:
            import traceback;traceback.print_exc();self.state.update(ready=False,error=str(e))
        finally:
            if rec:self.submit(rec.finish('aborted'))
            if sim:sim.close()
            self.stop.set();renderer.join(timeout=10);self.jobs.shutdown(wait=True)

def main():
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--scene-id',required=True);p.add_argument('--port',type=int,default=8767);p.add_argument('--host',default='0.0.0.0');p.add_argument('--token-file',type=Path,default=Path('runs/server.token'));a=p.parse_args()
    token=a.token_file.read_text().strip();uvicorn.run(create_app(AirportService(a.root,a.scene_id),token),host=a.host,port=a.port,log_level='warning')
if __name__=='__main__':main()
