"""Small synthetic control probe of the simulation HTTP API, not a demonstration."""
import argparse
import copy
import json
from pathlib import Path
import time
import uuid
import numpy as np
import requests


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--server',default='http://100.104.0.108:8765')
    p.add_argument('--token-file',type=Path,default=Path('tmp/r1pro_server.token'))
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();s=requests.Session()
    s.headers['Authorization']='Bearer '+a.token_file.read_text().strip()
    def call(path, data=None):
        r=s.get(a.server+path,timeout=3) if data is None else s.post(a.server+path,json=data,timeout=3)
        r.raise_for_status();return r
    def state():return call('/api/state').json()
    initial=state()
    if initial.get('source')!='simulation' or not initial.get('ready') or initial.get('recording') or initial.get('input_fresh'):
        raise RuntimeError('Need ready, idle simulation with no active recording/controller')
    a.output.mkdir(parents=True,exist_ok=False)
    samples=[];checks={};seq=0;cid='api-probe-'+uuid.uuid4().hex
    packet={'client_id':cid,'seq':0,'buttons':{},'hands':{
        side:{'position':[x,-0.2,-0.4],'quaternion_xyzw':[0,0,0,1],'grip':0.,'trigger':0.}
        for side,x in [('left',-0.2),('right',0.2)]}}
    def phase(name,duration,send=True):
        nonlocal seq
        end=time.monotonic()+duration
        while time.monotonic()<end:
            if send:
                packet['seq']=seq;seq+=1;call('/api/input',packet)
            value=state();samples.append({'phase':name,'time':time.monotonic(),**value})
            time.sleep(0.05)
        value=state()
        (a.output/(name+'.jpg')).write_bytes(call('/api/frame').content)
        return value
    def diff(x,y,indices):return float(np.max(np.abs(np.array(x['action'])[indices]-np.array(y['action'])[indices])))
    try:
        if initial['paused']:call('/api/command/pause',{})
        call('/api/command/reset',{});time.sleep(0.4)
        base=phase('baseline',0.3)
        for side,offset in [('left',0),('right',8)]:
            hand=packet['hands'][side];hand['grip']=1.
            opened=phase(side+'_open',0.7)
            hand['position'][0]+=(-0.04 if side=='left' else 0.04)
            moved=phase(side+'_move',1.2)
            checks[side+'_arm_target_changed']=diff(opened,moved,slice(offset,offset+7))>0.005
            other=8-offset
            checks[side+'_other_arm_held']=diff(opened,moved,slice(other,other+7))<1e-6
            hand['trigger']=1.
            closed=phase(side+'_close',1.3)
            checks[side+'_gripper_closed']=opened['state'][offset+7]-closed['state'][offset+7]>0.03
            hand['grip']=0.;hand['position'][0]+=0.2
            released=phase(side+'_release',0.6)
            checks[side+'_release_holds_target']=diff(closed,released,slice(offset,offset+8))<1e-6
        before=phase('before_timeout',0.2)
        after=phase('timeout',0.7,send=False)
        checks['stale_input_detected']=not after['input_fresh']
        checks['timeout_holds_targets']=diff(before,after,slice(None))<1e-6
        call('/api/command/pause',{});time.sleep(0.2)
        paused=state();packet['hands']['left']['grip']=1.
        packet['hands']['left']['position'][2]-=0.04
        after=phase('paused_input',0.6)
        checks['pause_holds_targets']=after['paused'] and diff(paused,after,slice(None))<1e-6
        checks['joint_state_followed_target']=max(abs(np.array(after['state'])-np.array(after['action'])))<0.03
    finally:
        # Release both clutches, then leave the simulation paused at the tested pose.
        for hand in packet['hands'].values():hand['grip']=0.
        packet['seq']=seq
        try:
            call('/api/input',packet)
            if not state()['paused']:call('/api/command/pause',{})
        finally:
            (a.output/'states.json').write_text(json.dumps(samples,indent=2))
            checks={k:bool(v) for k,v in checks.items()}
            report={'kind':'synthetic simulation control probe, not training data','checks':checks,
                    'passed':bool(checks) and all(checks.values()),'server_left_paused':True}
            (a.output/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))

if __name__=='__main__':main()
