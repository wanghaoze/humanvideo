"""Forward fresh Oculus Reader APK packets via authenticated HTTP to simulation."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import threading
import time
import uuid

import numpy as np
import requests
from scipy.spatial.transform import Rotation


def parse_line(line):
    # Wire format documented/implemented by jborbik/oculus_reader, Apache-2.0.
    if "wE9ryARX: " in line:line=line.split("wE9ryARX: ",1)[1]
    transforms,button_text=line.strip().split("&",1)
    matrices={}
    for item in transforms.split("|"):
        if ":" not in item:continue
        key,values=item.split(":",1)
        a=np.array([float(v) for v in values.split()],dtype=float)
        if key in ("l","r") and a.shape==(16,) and np.isfinite(a).all():
            matrix=a.reshape(4,4)
            if not np.allclose(matrix[3],[0,0,0,1],atol=1e-3):continue
            if not np.allclose(matrix[:3,:3].T@matrix[:3,:3],np.eye(3),atol=0.03):continue
            matrices[key]=matrix
    buttons={}
    for part in button_text.split(","):
        fields=part.strip().split()
        if len(fields)==1:buttons[fields[0]]=True
        elif len(fields)>1:buttons[fields[0]]=float(fields[1])
    hands={}
    for key,side,prefix in (("l","left","L"),("r","right","R")):
        if key not in matrices:continue
        m=matrices[key]
        # Pinned Quest 3 beta gates l/r matrices on the opposite hand's tracking
        # flag (Remotes.h). Never consume a pose unless its own hand is tracked.
        if not buttons.get(prefix, False):continue
        # An identity matrix is an observed uninitialized pose from this APK.
        if np.allclose(m, np.eye(4), atol=1e-7):continue
        hands[side]={"position":m[:3,3].tolist(),
                     "quaternion_xyzw":Rotation.from_matrix(m[:3,:3]).as_quat().tolist(),
                     "grip":float(buttons.get(side+"Grip",buttons.get(prefix+"G",False))),
                     "trigger":float(buttons.get(side+"Trig",buttons.get(prefix+"Tr",False)))}
    if not hands:raise ValueError("no valid controller poses")
    return {"hands":hands,"buttons":{b:buttons.get(b) is True for b in ("A","B","X","Y")}}


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--server",required=True,help="http(s)://server:8765")
    p.add_argument("--token",default=os.environ.get("R1PRO_TOKEN"))
    p.add_argument("--serial",required=True,help="adb serial or Quest IP:5555")
    p.add_argument("--adb",default="adb")
    p.add_argument("--check-only",action="store_true")
    p.add_argument("--package",default="org.humanvideo.questreader",choices=["org.humanvideo.questreader","com.rail.oculus.teleop"])
    p.add_argument("--check-seconds",type=int,default=30,help="Diagnostic duration (1-120 seconds); check-only mode stops the app afterward")
    a=p.parse_args()
    if not 1 <= a.check_seconds <= 120:p.error("--check-seconds must be 1-120")
    if not a.token and not a.check_only:p.error("set R1PRO_TOKEN or --token")
    adb=[a.adb,"-s",a.serial]
    model=subprocess.check_output(adb+["shell","getprop","ro.product.model"],text=True).strip()
    brand=subprocess.check_output(adb+["shell","getprop","ro.product.manufacturer"],text=True).strip()
    print(json.dumps({"device_model":model,"manufacturer":brand,"input":"Oculus Reader OpenXR"}),flush=True)
    launch=adb+["shell","am","start","-n",a.package+"/com.rail.oculus.teleop.MainActivity"]
    if a.check_only:launch += ["--ei","diagnostic_timeout_seconds",str(a.check_seconds)]
    subprocess.run(launch,check=True)
    child=subprocess.Popen(adb+["logcat","-v","raw","-T","0","-s","wE9ryARX:I","*:S"],
        stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,encoding="utf-8",errors="replace",
        creationflags=subprocess.CREATE_NO_WINDOW if os.name=="nt" else 0)
    lock=threading.Lock();latest={};stop=threading.Event()
    def consume():
        for line in child.stdout:
            if stop.is_set():break
            try:packet=parse_line(line)
            except (ValueError,TypeError,IndexError):continue
            with lock:latest.update(packet=packet,received=time.monotonic(),seq=latest.get("seq",-1)+1)
    thread=threading.Thread(target=consume,daemon=True);thread.start()
    session=requests.Session();session.headers["Authorization"]="Bearer "+(a.token or "")
    client_id=uuid.uuid4().hex;last_seq=-1;last_notice=0.;started=time.monotonic()
    try:
        while child.poll() is None:
            if a.check_only and time.monotonic()-started >= a.check_seconds:
                print("Diagnostic interval complete; closing app",flush=True)
                break
            with lock:item=latest.copy()
            now=time.monotonic()
            if item and item["seq"]!=last_seq and now-item["received"]<0.2:
                packet={**item["packet"],"seq":item["seq"],"client_id":client_id}
                if a.check_only:
                    if now-last_notice>1:print(json.dumps(packet),flush=True);last_notice=now
                else:
                    try:
                        response=session.post(a.server.rstrip("/")+"/api/input",json=packet,timeout=0.3)
                        response.raise_for_status()
                    except requests.RequestException as e:
                        if now-last_notice>2:print("Connection:",str(e),flush=True);last_notice=now
                last_seq=item["seq"]
            elif now-last_notice>5:
                print("Waiting for fresh tracked controller data; hold controllers in view",flush=True);last_notice=now
            stop.wait(1/40)
        if child.poll() is not None:raise RuntimeError("adb logcat stopped: "+child.stderr.read())
    except KeyboardInterrupt:pass
    finally:
        stop.set();child.terminate();child.wait(timeout=5);session.close()
        if a.check_only:subprocess.run(adb+["shell","am","force-stop",a.package],timeout=10)


if __name__=="__main__":main()
