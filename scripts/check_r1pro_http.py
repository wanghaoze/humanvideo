"""Exercise the running simulator via HTTP; never labels a test as successful grasp."""
import argparse
import json
from pathlib import Path
import time
import uuid
import numpy as np
import requests


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--url", default="http://127.0.0.1:8765")
    p.add_argument("--token-file", type=Path, default=Path("runs/server.token"))
    p.add_argument("--output", type=Path, default=Path("runs/http_acceptance.json"))
    args = p.parse_args()
    client = requests.Session()
    client.headers["Authorization"] = "Bearer " + args.token_file.read_text().strip()

    def get(path):
        r = client.get(args.url + path, timeout=10)
        r.raise_for_status()
        return r

    def command(name):
        r = client.post(args.url + "/api/command/" + name, timeout=10)
        r.raise_for_status()

    def until(predicate, timeout=20):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            state = get("/api/state").json()
            if state.get("error"):
                raise RuntimeError(state["error"])
            if predicate(state):
                return state
            time.sleep(0.1)
        raise TimeoutError(state)

    until(lambda s: s["ready"])
    assert requests.get(args.url + "/api/state", timeout=10).status_code == 401
    assert "html" in get("/vr").text.lower()
    assert get("/api/frame").content[:2] == b"\xff\xd8"
    command("reset")
    time.sleep(0.3)
    initial = get("/api/state").json()
    command("record")
    until(lambda s: s["recording"])
    cid = "acceptance-" + uuid.uuid4().hex
    durations = []
    for i in range(40):
        packet = {"client_id": cid, "seq": i, "hands": {"left": {
            "position": [min(i / 15, 1) * 0.08, 1, 0],
            "quaternion_xyzw": [0, 0, 0, 1], "grip": 0.8, "trigger": 0.2}}, "buttons": {}}
        r = client.post(args.url + "/api/input", json=packet, timeout=10)
        r.raise_for_status()
        durations.append(get("/api/state").json().get("processing_ms", 0))
        time.sleep(0.05)
    moved = get("/api/state").json()
    change = float(np.max(np.abs(np.array(moved["action"]) - initial["action"])))
    assert change > 1e-4, "Controller packet did not change joint targets"
    assert client.post(args.url + "/api/input", json=packet, timeout=10).status_code == 409
    stale = until(lambda s: not s["input_fresh"])
    command("save_unlabeled")
    saved = until(lambda s: not s["recording"] and s.get("last_episode"))
    report = {"ready": True, "unauthorized_rejected": True, "duplicate_rejected": True,
              "jpeg": True, "vr_page": True, "input_expired": not stale["input_fresh"],
              "max_action_change": change, "last_episode": saved["last_episode"],
              "recording_cycle_ms_median": float(np.median(durations)),
              "recording_cycle_ms_p95": float(np.percentile(durations, 95)),
              "source": "simulation", "quest_hardware_tested": False,
              "successful_grasp_tested": False}
    command("reset")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
