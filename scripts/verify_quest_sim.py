"""Bounded real Quest input -> existing MuJoCo HTTP service validation."""
import argparse
import json
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
import uuid
import numpy as np
import requests

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from r1pro_teleop.quest_bridge import parse_line


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--server',default='http://100.104.0.108:8765')
    p.add_argument('--token-file',type=Path,default=Path('tmp/r1pro_server.token'))
    p.add_argument('--adb',default='deliverables/quest3/adb/platform-tools/adb.exe')
    p.add_argument('--serial',default='2G97C5ZH5D006C')
    p.add_argument('--seconds',type=int,default=30)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if not 10<=a.seconds<=60:p.error('seconds must be 10-60')
    adb=[a.adb,'-s',a.serial];package='org.humanvideo.questreader'
    def device(*args):return subprocess.run(adb+list(args),capture_output=True,text=True,check=True,timeout=15)
    if 'mWakefulness=Awake' not in device('shell','dumpsys','power').stdout:
        raise RuntimeError('Wake headset and wear it first; app not started')
    s=requests.Session();s.headers['Authorization']='Bearer '+a.token_file.read_text().strip()
    def call(path,data=None):
        r=s.get(a.server+path,timeout=2) if data is None else s.post(a.server+path,json=data,timeout=2)
        r.raise_for_status();return r
    def state():return call('/api/state').json()
    before=state()
    if before.get('source')!='simulation' or not before.get('ready') or before.get('recording') or before.get('input_fresh'):
        raise RuntimeError('Need idle simulation with no recording or other controller')
    a.output.mkdir(parents=True,exist_ok=False)
    lock=threading.Lock();latest={};inputs=[];states=[];errors=[];child=None
    sent=0;seq=0;cid='quest-live-'+uuid.uuid4().hex;stopped=False
    try:
        call('/api/command/reset',{});time.sleep(0.4)
        before=state()
        launch=device('shell','am','start','-W','-n',package+'/com.rail.oculus.teleop.MainActivity',
                      '--ei','diagnostic_timeout_seconds',str(a.seconds+5))
        (a.output/'launch.txt').write_text(launch.stdout)
        pid=device('shell','pidof',package).stdout.strip()
        if not pid.isdigit():raise RuntimeError('No unique app PID')
        child=subprocess.Popen(adb+['logcat','-v','raw','--pid',pid,'-T','0','-s','wE9ryARX:I','*:S'],
            stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,encoding='utf-8',errors='replace')
        def consume():
            for line in child.stdout:
                try:packet=parse_line(line)
                except (ValueError,TypeError,IndexError):continue
                with lock:latest.update(packet=packet,received=time.monotonic(),serial=latest.get('serial',0)+1)
        threading.Thread(target=consume,daemon=True).start()
        start=time.monotonic();last_serial=-1;next_state=0.;next_frame=0.
        while time.monotonic()-start<a.seconds:
            now=time.monotonic();elapsed=now-start
            with lock:item=latest.copy()
            if item and item['serial']!=last_serial and now-item['received']<0.15:
                # Suppress recording/success/pause shortcuts during validation.
                packet={'hands':item['packet']['hands'],'buttons':{},'client_id':cid,'seq':seq}
                seq+=1;last_serial=item['serial']
                t=time.monotonic();call('/api/input',packet);sent+=1
                inputs.append({'elapsed':elapsed,'http_ms':(time.monotonic()-t)*1000,**packet})
            if elapsed>=next_state:
                states.append({'elapsed':elapsed,**state()});next_state=elapsed+0.2
            if elapsed>=next_frame:
                (a.output/f'frame_{elapsed:07.2f}.jpg').write_bytes(call('/api/frame').content)
                next_frame=elapsed+0.5
            time.sleep(0.025)
        # Stop transmitting; verify stale input leaves targets unchanged.
        time.sleep(0.4);stale1=state();time.sleep(0.4);stale2=state()
        stale_ok=not stale2['input_fresh'] and np.allclose(stale1['action'],stale2['action'],atol=1e-7)
    except Exception as e:
        errors.append(str(e));raise
    finally:
        if child:
            child.terminate()
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:child.kill();child.wait()
        try:
            device('shell','am','force-stop',package)
            stopped=subprocess.run(adb+['shell','pidof',package],capture_output=True,timeout=5).returncode==1
            if not state()['paused']:call('/api/command/pause',{})
        except Exception as e:errors.append('cleanup: '+str(e))
        report={'source':'live Quest -> HTTP -> MuJoCo simulation','sent_packets':sent,'errors':errors,
                'app_stopped':stopped,'timeout_holds_targets':bool(locals().get('stale_ok',False)),'hands':{}}
        for side,offset in [('left',0),('right',8)]:
            hand=[x['hands'][side] for x in inputs if side in x['hands']]
            if hand and states:
                q=np.array([x['state'] for x in states])
                report['hands'][side]={'grip_range':[min(x['grip'] for x in hand),max(x['grip'] for x in hand)],
                    'trigger_range':[min(x['trigger'] for x in hand),max(x['trigger'] for x in hand)],
                    'arm_joint_max_range_rad':float(np.max(np.ptp(q[:,offset:offset+7],axis=0))),
                    'gripper_opening_range_m':[float(q[:,offset+7].min()),float(q[:,offset+7].max())]}
        if inputs:report['http_ms_median']=float(np.median([x['http_ms'] for x in inputs]))
        report['motion_checks']={}
        for side in ('left','right'):
            h=report['hands'].get(side,{})
            report['motion_checks'][side]=bool(h and h['grip_range'][1]>=0.5 and
                h['arm_joint_max_range_rad']>0.005 and
                h['gripper_opening_range_m'][1]-h['gripper_opening_range_m'][0]>0.01)
        report['passed']=sent>20 and not errors and stopped and report['timeout_holds_targets'] and all(report['motion_checks'].values())
        for filename,data in [('inputs.json',inputs),('states.json',states),('report.json',report)]:
            (a.output/filename).write_text(json.dumps(data,indent=2),encoding='utf-8')
        print(json.dumps(report,indent=2))

if __name__=='__main__':main()
