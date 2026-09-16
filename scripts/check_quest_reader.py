"""Bounded Quest-only diagnostic. Never connects to a robot or simulation."""
import argparse
import json
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from r1pro_teleop.quest_bridge import parse_line


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--adb', required=True)
    p.add_argument('--serial', required=True)
    p.add_argument('--seconds', type=int, default=20)
    p.add_argument('--output', type=Path, required=True)
    a=p.parse_args()
    if not 5 <= a.seconds <= 60:p.error('seconds must be 5-60')
    adb=[a.adb,'-s',a.serial]
    package='org.humanvideo.questreader'
    def run(*args, **kwargs):
        return subprocess.run(adb+list(args),timeout=15,check=True,**kwargs)
    power=run('shell','dumpsys','power',capture_output=True,text=True).stdout
    if 'mWakefulness=Awake' not in power:
        raise RuntimeError('Headset is asleep. Wake it and keep it worn before testing; app was not launched.')
    a.output.mkdir(parents=True,exist_ok=False)
    run('shell','pm','path',package,capture_output=True)
    child=None; samples=[]; lines=[]; captured=False; stop_ok=False
    try:
        launch=run('shell','am','start','-W','-n',package+'/com.rail.oculus.teleop.MainActivity',
            '--ei','diagnostic_timeout_seconds',str(a.seconds),capture_output=True,text=True)
        (a.output/'launch.txt').write_text(launch.stdout+launch.stderr,encoding='utf-8')
        pid=''
        for _ in range(20):
            found=subprocess.run(adb+['shell','pidof',package],capture_output=True,text=True,timeout=5)
            pid=found.stdout.strip()
            if pid.isdigit():break
            time.sleep(0.2)
        if not pid.isdigit():raise RuntimeError('No unique app process after launch')
        child=subprocess.Popen(adb+['logcat','-v','raw','--pid',pid],
            stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,encoding='utf-8',errors='replace')
        q=queue.Queue()
        def consume():
            for line in child.stdout:q.put(line)
        threading.Thread(target=consume,daemon=True).start()
        started=time.monotonic()
        while time.monotonic()-started < a.seconds:
            try:
                line=q.get(timeout=0.1);lines.append(line)
                try:
                    packet=parse_line(line)
                    samples.append({'elapsed':time.monotonic()-started,**packet})
                except (ValueError,TypeError,IndexError):pass
            except queue.Empty:pass
            if not captured and time.monotonic()-started>5:
                shot=run('exec-out','screencap','-p',capture_output=True).stdout
                if shot.startswith(b'\x89PNG'):(a.output/'view.png').write_bytes(shot)
                else:(a.output/'screenshot_unavailable.txt').write_text(shot.decode('utf-8',errors='replace') or 'No image returned by Android screencap',encoding='utf-8')
                captured=True
        print(f'Received {len(samples)} valid pose packets')
    finally:
        if child:
            child.terminate()
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:child.kill();child.wait()
        try:
            run('shell','am','force-stop',package,capture_output=True)
            check=subprocess.run(adb+['shell','pidof',package],capture_output=True,timeout=10)
            stop_ok=check.returncode==1 and not check.stdout.strip()
        finally:
            (a.output/'app.log').write_text(''.join(lines),encoding='utf-8')
            (a.output/'samples.json').write_text(json.dumps(samples,indent=2),encoding='utf-8')
            summary={'valid_packets':len(samples),'both_hands_packets':sum(len(x['hands'])==2 for x in samples),
                'app_stopped':stop_ok,'hardware_visual_confirmation':'pending','hands':{}}
            for side in ('left','right'):
                hand=[x['hands'][side] for x in samples if side in x['hands']]
                if hand:
                    summary['hands'][side]={'samples':len(hand),
                        'position_axis_ranges_m':[max(x['position'][i] for x in hand)-min(x['position'][i] for x in hand) for i in range(3)],
                        'trigger_range':[min(x['trigger'] for x in hand),max(x['trigger'] for x in hand)]}
            (a.output/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
            print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
