"""HTTP viewer/control service for MuJoCo only. No ROS or hardware outputs."""
import argparse
import io
import json
import os
from pathlib import Path
import queue
import secrets
import threading
import time
import uuid
from contextlib import asynccontextmanager

from fastapi import FastAPI,HTTPException,Request
from fastapi.responses import HTMLResponse,Response
from fastapi.staticfiles import StaticFiles
from PIL import Image,ImageDraw
import uvicorn


class Service:
    def __init__(self,scene,raw):
        self.scene,self.raw=scene,raw
        self.lock=threading.Lock();self.commands=queue.Queue(maxsize=20)
        self.incoming=None;self.owner=None;self.owner_time=0.;self.sequence=-1
        self.stop=threading.Event();self.jpeg=None;self.views={};self.snapshot=None
        self.state={"ready":False,"recording":False,"source":"simulation"}
        self.session_id=uuid.uuid4().hex

    def render_views(self):
        # Dedicated MuJoCo data/context: slow video must not delay control ticks.
        from .core import Simulation
        import mujoco
        view_sim=None
        try:
            view_sim=Simulation(self.scene,width=640,height=480)
            while not self.stop.is_set():
                with self.lock:snapshot=self.snapshot
                if snapshot is None:
                    self.stop.wait(.02);continue
                started=time.monotonic()
                view_sim.data.qpos[:]=snapshot[0]
                view_sim.data.qvel[:]=snapshot[1]
                view_sim.data.time=snapshot[2]
                mujoco.mj_forward(view_sim.model,view_sim.data)
                images={name:Image.fromarray(view_sim.image(name)) for name in
                        ("overview","front","left_wrist","right_wrist")}
                grid=Image.new("RGB",(640,480))
                for i,(name,picture) in enumerate(images.items()):
                    tile=picture.resize((320,240));draw=ImageDraw.Draw(tile)
                    draw.rectangle((0,0,150,20),fill=(15,25,35))
                    draw.text((6,4),name,fill="white")
                    grid.paste(tile,((i%2)*320,(i//2)*240))
                images["grid"]=grid;views={}
                for name,picture in images.items():
                    buf=io.BytesIO();picture.save(buf,format="JPEG",quality=85)
                    views[name]=buf.getvalue()
                with self.lock:
                    self.views=views;self.jpeg=views["overview"]
                    self.state.update(last_frame_monotonic=snapshot[3],
                        render_ms=round((time.monotonic()-started)*1000,1),
                        render_sim_time=snapshot[2])
                self.stop.wait(max(0,.1-(time.monotonic()-started)))
        except Exception as e:
            with self.lock:self.state["render_error"]=str(e)
        finally:
            if view_sim:view_sim.close()

    def run(self):
        from .core import Simulation,Recorder
        sim=None;rec=None;previous_buttons={};handled=None
        render_thread=threading.Thread(target=self.render_views,name="monitor-renderer",daemon=True)
        render_thread.start()
        try:
            sim=Simulation(self.scene,width=1280,height=720,render=False)
            while not self.stop.is_set():
                started=time.monotonic()
                with self.lock:incoming=self.incoming
                if incoming and incoming["id"]!=handled:
                    if incoming["client_id"]!=getattr(sim,"client_id",None):
                        sim.disconnect();sim.client_id=incoming["client_id"]
                    sim.input(incoming["packet"],now=incoming["received"])
                    handled=incoming["id"]
                    buttons=incoming["packet"].get("buttons",{})
                    if started-incoming["received"]<0.25:
                        for b,cmd in (("X","record"),("A","save_success"),("B","pause")):
                            if buttons.get(b) and not previous_buttons.get(b):
                                try:self.commands.put_nowait(cmd)
                                except queue.Full:pass
                    previous_buttons=buttons
                while not self.commands.empty():
                    cmd=self.commands.get_nowait()
                    if cmd=="record" and rec is None:
                        if sim.renderer is None:
                            import mujoco
                            sim.renderer=mujoco.Renderer(sim.model,height=sim.height,width=sim.width)
                        rec=Recorder(self.raw,sim,session_id=self.session_id)
                    elif cmd in ("save_success","save_failure","save_unlabeled") and rec is not None:
                        result=cmd.removeprefix("save_")
                        path=rec.finish(result);rec=None
                        with self.lock:self.state["last_episode"]=str(path)
                    elif cmd in ("reset", "reset_paused"):
                        if rec:rec.finish("failure");rec=None
                        sim.reset();sim.paused=(cmd=="reset_paused");handled=None;previous_buttons={}
                        with self.lock:self.incoming=None;self.owner=None;self.sequence=-1
                    elif cmd=="pause":
                        sim.paused=not sim.paused
                        sim.anchors.clear()
                sim.update_control()
                # State and all three RGB images refer to this same physics state.
                if rec:
                    age=max(0,started-sim.last_packet_time) if sim.last_packet_time else -1.
                    rec.append(sim,sim.images(),age)
                with self.lock:
                    self.snapshot=(sim.data.qpos.copy(),sim.data.qvel.copy(),float(sim.data.time),time.monotonic())
                    self.state.update(ready=True,recording=rec is not None,
                        frames=rec.count if rec else 0,paused=sim.paused,sim_time=float(sim.data.time),
                        input_fresh=started-sim.last_packet_time<0.25,
                        state=sim.state().tolist(),action=sim.target.tolist(),
                        tray_xyz=sim.data.body("tray").xpos.tolist(),physics="MuJoCo CPU; GPU rendering")
                sim.step()
                elapsed=time.monotonic()-started
                with self.lock:self.state["processing_ms"]=round(elapsed*1000,1)
                self.stop.wait(max(0,1/sim.fps-elapsed))
        except Exception as e:
            import traceback
            traceback.print_exc()
            with self.lock:self.state.update(ready=False,error=str(e))
        finally:
            if rec:rec.finish("unlabeled")
            if sim:sim.close()
            self.stop.set();render_thread.join(timeout=10)


def create_app(service,token):
    @asynccontextmanager
    async def lifespan(app):
        thread=threading.Thread(target=service.run,name="mujoco-owner",daemon=True)
        thread.start()
        yield
        service.stop.set();thread.join(timeout=10)
    app=FastAPI(lifespan=lifespan)
    app.mount("/static",StaticFiles(directory=Path(__file__).parent/"static"),name="static")
    def auth(request):
        value=request.headers.get("authorization","")
        if not secrets.compare_digest(value,"Bearer "+token):raise HTTPException(401,"Invalid token")
    @app.get("/",response_class=HTMLResponse)
    def index():return (Path(__file__).parent/getattr(service,"viewer_file","viewer.html")).read_text(encoding="utf-8")
    @app.get("/vr",response_class=HTMLResponse)
    def vr():return (Path(__file__).parent/"vr.html").read_text(encoding="utf-8")
    @app.get("/api/state")
    def state(request:Request):
        auth(request)
        with service.lock:
            result=dict(service.state)
            last = result.pop('last_frame_monotonic', None)
            result['frame_age_s'] = time.monotonic()-last if last is not None else None
            result["input_fresh"]=bool(service.owner and time.monotonic()-service.owner_time<0.25)
            return result
    @app.get("/api/frame")
    def frame(request:Request,view:str="overview"):
        auth(request)
        if view not in ("overview","front","left_wrist","right_wrist","grid"):
            raise HTTPException(400,"Unknown view")
        with service.lock:
            jpeg=service.views.get(view)
            stamp=service.state.get("last_frame_monotonic")
        if jpeg is None:raise HTTPException(503,"Renderer starting")
        return Response(jpeg,media_type="image/jpeg",headers={"Cache-Control":"no-store",
                        "X-Frame-Age":str(time.monotonic()-stamp if stamp else 999)})
    @app.post("/api/input")
    async def input_packet(request:Request):
        auth(request)
        from .core import validate_packet
        if int(request.headers.get("content-length","0"))>8192:raise HTTPException(413)
        packet=await request.json()
        try:
            validate_packet(packet)
            cid=packet["client_id"]
            if not isinstance(cid,str) or not 1<=len(cid)<=80:raise ValueError("bad client_id")
        except (KeyError,ValueError,TypeError):raise HTTPException(422,"Invalid controller packet")
        now=time.monotonic()
        with service.lock:
            if service.owner!=cid:
                if service.owner and now-service.owner_time<1:raise HTTPException(409,"Another controller is active")
                service.owner=cid;service.sequence=-1
            if packet["seq"]<=service.sequence:raise HTTPException(409,"Old or duplicate packet")
            service.sequence=packet["seq"];service.owner_time=now
            service.incoming={"id":(cid,packet["seq"]),"client_id":cid,"packet":packet,"received":now}
        return {"accepted":True}
    @app.post("/api/command/{command}")
    def command(command:str,request:Request):
        auth(request)
        if command not in getattr(service,"allowed_commands",("record","save_success","save_failure","save_unlabeled","reset","reset_paused","pause")):
            raise HTTPException(400,"Unknown command")
        try:service.commands.put_nowait(command)
        except queue.Full:raise HTTPException(429,"Command queue full")
        return {"queued":command}
    return app


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--scene",type=Path,default=Path("outputs/r1pro_tray_scene/scene_direct_grasp_720p.xml"))
    p.add_argument("--raw",type=Path,default=Path("runs/r1pro_raw"))
    p.add_argument("--host",default="0.0.0.0")
    p.add_argument("--port",type=int,default=8765)
    p.add_argument("--token",default=os.environ.get("R1PRO_TOKEN"))
    p.add_argument("--certfile");p.add_argument("--keyfile")
    args=p.parse_args()
    token=args.token or secrets.token_urlsafe(24)
    if not args.token:print("Set this token in viewer/bridge:",token,flush=True)
    print("Simulation server; no real robot control. Open browser at port",args.port,flush=True)
    app=create_app(Service(args.scene,args.raw),token)
    uvicorn.run(app,host=args.host,port=args.port,log_level="warning",
                ssl_certfile=args.certfile,ssl_keyfile=args.keyfile)


if __name__=="__main__":main()
