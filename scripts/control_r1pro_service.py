"""Start/inspect/stop this project's simulation service inside an active Conda env."""
import argparse
import json
import os
from pathlib import Path
import secrets
import signal
import socket
import subprocess
import sys
import time
import urllib.request


def health(record):
    if not alive(record): return {"running": False, "healthy": False}
    try:
        token = Path(record['token_file']).read_text().strip()
        request = urllib.request.Request(f"http://127.0.0.1:{record['port']}/api/state",
                                         headers={'Authorization': 'Bearer ' + token})
        with urllib.request.urlopen(request, timeout=2) as response:
            state = json.load(response)
        age=state.get('frame_age_s')
        return {"running": True, "healthy": bool(state.get('ready')) and isinstance(age,(int,float)) and age<2,
                "recording": state.get('recording'), "processing_ms": state.get('processing_ms'),
                "error": state.get('error') or state.get('render_error')}
    except (OSError, ValueError, KeyError) as error:
        return {"running": True, "healthy": False, "error": str(error)}

ROOT = Path(__file__).resolve().parents[1]
RUNS = ROOT / "runs"
RECORD = RUNS / "server_process.json"


def alive(record):
    try:
        pid = int(record["pid"])
        cmd = Path(f"/proc/{pid}/cmdline").read_bytes()
        return b"r1pro_teleop.server" in cmd and Path(f"/proc/{pid}/cwd").resolve() == ROOT
    except (OSError, KeyError):
        return False


def main():
    p = argparse.ArgumentParser()
    p.add_argument("command", choices=["start", "status", "stop"])
    a = p.parse_args()
    record = json.loads(RECORD.read_text()) if RECORD.exists() else {}
    if a.command == "status":
        print(json.dumps({**record, **health(record)}, indent=2))
        return
    if a.command == "stop":
        if alive(record):
            os.kill(int(record["pid"]), signal.SIGTERM)
            for _ in range(100):
                if not alive(record):
                    break
                time.sleep(0.1)
        print(json.dumps({"running": alive(record)}))
        return
    prefix = os.environ.get("CONDA_PREFIX")
    if not prefix or Path(prefix).resolve() != Path(sys.prefix).resolve():
        raise RuntimeError("Run with conda run -n r1pro-tray python, or activate that Conda environment")
    if alive(record):
        print(json.dumps({**record, "already_running": True}, indent=2))
        return
    with socket.socket() as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", 8765))
    RUNS.mkdir(exist_ok=True)
    token_file = RUNS / "server.token"
    if not token_file.exists():
        with os.fdopen(os.open(token_file, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600), "w") as f:
            f.write(secrets.token_urlsafe(24) + "\n")
    token_file.chmod(0o600)
    from select_r1pro_gpu import select
    gpu = select(int(os.environ.get("R1PRO_GPU", "0")))
    env = dict(os.environ, R1PRO_TOKEN=token_file.read_text().strip(), MUJOCO_GL="egl",
               MUJOCO_EGL_DEVICE_ID=str(gpu["egl_index"]), CUDA_VISIBLE_DEVICES=gpu["uuid"],
               CUDA_DEVICE_ORDER="PCI_BUS_ID")
    with (RUNS / "server.log").open("ab") as log:
        process = subprocess.Popen([sys.executable, "-m", "r1pro_teleop.server"], cwd=ROOT,
                                   env=env, stdin=subprocess.DEVNULL, stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
    record = {"pid": process.pid, "conda_prefix": prefix, "port": 8765,
              "gpu": gpu, "token_file": str(token_file), "log": str(RUNS / "server.log")}
    RECORD.write_text(json.dumps(record, indent=2))
    for _ in range(100):
        status = health(record)
        if status['healthy'] or not status['running']: break
        time.sleep(.2)
    print(json.dumps({**record, **status}, indent=2))
    if not status['healthy']: raise SystemExit('Service process started but is not healthy; inspect server.log')


if __name__ == "__main__":
    main()
