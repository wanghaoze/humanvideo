"""Loopback-only authenticated USB relay for Quest Browser WebXR + video."""
import argparse
import json
from pathlib import Path
import secrets
import threading
import time

from fastapi import FastAPI,Request,HTTPException
from fastapi.responses import HTMLResponse,Response,RedirectResponse,JSONResponse
import requests
from starlette.concurrency import run_in_threadpool
import uvicorn


def create_app(server, token, session_key, output, port=8766):
    app=FastAPI();local=threading.local();lock=threading.Lock()
    stats={'started':time.time(),'telemetry':None,'frames':0,'accepted_inputs':0,'errors':0}
    samples=[]
    def upstream(path,method='GET',data=None):
        if not hasattr(local,'http'):
            local.http=requests.Session();local.http.headers['Authorization']='Bearer '+token
        try:
            r=local.http.request(method,server.rstrip('/')+path,json=data,timeout=2)
            if r.status_code>=400:raise HTTPException(r.status_code,'Simulation request rejected')
            return r
        except requests.RequestException:
            with lock:stats['errors']+=1
            raise HTTPException(502,'Simulation server unreachable')
    @app.middleware('http')
    async def protect(request:Request,call_next):
        if request.url.path.startswith('/session/') or request.url.path=='/health':return await call_next(request)
        if not secrets.compare_digest(request.cookies.get('quest_session',''),session_key):
            return JSONResponse({'error':'Open the session link from the launcher'},status_code=401)
        if request.method=='POST' and request.headers.get('origin') not in (f'http://localhost:{port}',f'http://127.0.0.1:{port}'):
            return JSONResponse({'error':'Unexpected origin'},status_code=403)
        if int(request.headers.get('content-length','0'))>16384:return JSONResponse({'error':'Payload too large'},status_code=413)
        return await call_next(request)
    @app.get('/health')
    def health():return {'service':'humanvideo-quest-usb-relay','ready':True}
    @app.get('/session/{key}')
    def login(key:str):
        if not secrets.compare_digest(key,session_key):raise HTTPException(403)
        r=RedirectResponse('/vr',status_code=303)
        r.set_cookie('quest_session',session_key,httponly=True,samesite='strict',max_age=14400)
        r.headers['Referrer-Policy']='no-referrer';return r
    @app.get('/vr',response_class=HTMLResponse)
    def page():return (Path(__file__).parent/'quest_monitor.html').read_text(encoding='utf-8')
    @app.get('/static/aframe.min.js')
    def aframe():return Response((Path(__file__).parent/'static/aframe.min.js').read_bytes(),media_type='application/javascript')
    @app.get('/api/state')
    def state():return upstream('/api/state').json()
    @app.get('/api/frame')
    def frame(view:str="overview"):
        if view not in ("overview","front","left_wrist","right_wrist","grid"):raise HTTPException(400)
        r=upstream('/api/frame?view='+view)
        with lock:stats['frames']+=1
        return Response(r.content,media_type='image/jpeg',headers={'Cache-Control':'no-store','X-Frame-Age':r.headers.get('X-Frame-Age','999')})
    @app.post('/api/input')
    async def input_packet(request:Request):
        data=await request.json();data['buttons']={}
        r=await run_in_threadpool(upstream,'/api/input','POST',data)
        with lock:stats['accepted_inputs']+=1
        return r.json()
    @app.post('/api/command/{command}')
    def command(command:str):
        if command not in ('pause','reset','reset_paused'):raise HTTPException(400)
        return upstream('/api/command/'+command,'POST',{}).json()
    @app.post('/api/telemetry')
    async def telemetry(request:Request):
        data=await request.json()
        with lock:
            stats['telemetry']={'received':time.time(),**data}
            samples.append(stats['telemetry'])
            # Bound the log to roughly the last hour; no images or credentials.
            if len(samples)>3600:del samples[:600]
            (output/'telemetry.json').write_text(json.dumps(samples,indent=2),encoding='utf-8')
        return {'ok':True}
    @app.get('/api/relay-state')
    def relay_state():
        with lock:return dict(stats)
    return app


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--server',default='http://100.104.0.108:8765')
    p.add_argument('--token-file',type=Path,default=Path('tmp/r1pro_server.token'))
    p.add_argument('--session-file',type=Path,default=Path('runs/quest_webxr_session.json'))
    p.add_argument('--output',type=Path,default=Path('runs/quest_webxr_20260915'))
    p.add_argument('--port',type=int,default=8766)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    session=json.loads(a.session_file.read_text())
    app=create_app(a.server,a.token_file.read_text().strip(),session['key'],a.output,a.port)
    uvicorn.run(app,host='127.0.0.1',port=a.port,access_log=False,log_level='warning')

if __name__=='__main__':main()
