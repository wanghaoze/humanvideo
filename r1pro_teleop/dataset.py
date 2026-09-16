"""Validate raw synchronized episodes; export through the official LeRobot API."""
import argparse
import hashlib
import json
from pathlib import Path

import h5py
import numpy as np
from .core import NAMES, CAMERAS


def raw_digest(path):
    digest=hashlib.sha256()
    with open(path,"rb") as f:
        for block in iter(lambda:f.read(8*1024*1024),b""):digest.update(block)
    return digest.hexdigest()


def validate(path):
    with h5py.File(path,"r") as f:
        if f.attrs.get("schema") != "r1pro_sim_raw_v1" or f.attrs.get("source") != "simulation":
            raise ValueError(f"{path}: unexpected schema/source")
        n = int(f.attrs["num_frames"])
        fps = int(f.attrs["fps"])
        if n < 2 or json.loads(f.attrs["names"]) != NAMES:
            raise ValueError(f"{path}: too short or incompatible joint order")
        for key in ("state","action"):
            a=f[key][:]
            if a.shape!=(n,16) or a.dtype!=np.float32 or not np.isfinite(a).all():
                raise ValueError(f"{path}: invalid {key}")
            if np.any(a[:,[7,15]]<0) or np.any(a[:,[7,15]]>0.105):
                raise ValueError(f"{path}: gripper opening units/range invalid")
        t=f["timestamp"][:]
        if len(t)!=n or not np.allclose(t,np.arange(n)/fps,atol=1e-5):
            raise ValueError(f"{path}: irregular simulation timestamps")
        for cam in CAMERAS:
            a=f[f"images/{cam}"]
            if a.shape[0]!=n or a.ndim!=4 or a.shape[-1]!=3 or a.dtype!=np.uint8:
                raise ValueError(f"{path}: invalid {cam} images")
        return {"file":str(path),"frames":n,"fps":fps,"result":str(f.attrs["result"]),
                "session_id":str(f.attrs["session_id"]),
                "scene_sha256":str(f.attrs["scene_sha256"]),
                "controller_source":str(f.attrs.get("controller_source","unspecified")),
                "task_kind":str(f.attrs.get("task_kind","tray_transfer")),
                "success_operator":bool(f.attrs.get("success_operator",False)),
                "image_shape":list(f[f"images/{CAMERAS[0]}"].shape[1:]),
                "max_state_action_difference":float(np.max(abs(f["state"][:]-f["action"][:]))) }


def export(raw,output,repo_id,include_all=False,require_task_checks=False):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from importlib.metadata import version
    if version("lerobot") != "0.4.4":
        raise RuntimeError("Use the supplied lerobot==0.4.4 environment")
    output=Path(output).resolve()
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite {output}; choose a new output directory")
    paths=[p for p in sorted(Path(raw).glob("*.h5")) if not p.name.endswith(".partial.h5")]
    if not include_all:
        success_paths=[]
        for path in paths:
            with h5py.File(path,"r") as f:
                if f.attrs.get("result")=="success":success_paths.append(path)
        paths=success_paths
    reports=[validate(p) for p in paths]
    chosen=[(p,r) for p,r in zip(paths,reports) if include_all or r["result"]=="success"]
    if not chosen:
        raise ValueError("No completed success episodes. Label successful runs, or use --include-all for diagnostics only")
    if require_task_checks:
        if include_all: raise ValueError('Strict training export cannot include non-success diagnostic episodes')
        from .quality import inspect
        rejected = [inspect(p) for p, _ in chosen]
        rejected = [r for r in rejected if not r['training_candidate']]
        if rejected:
            raise ValueError('Quality/task checks rejected episodes; review quality report first: ' +
                             ', '.join(r['file'] for r in rejected))
    signatures={(r["fps"],tuple(r["image_shape"]),r["scene_sha256"],r["controller_source"],r["task_kind"]) for _,r in chosen}
    if len(signatures)!=1:
        raise ValueError("Do not merge incompatible cameras, fps or scene versions")
    fps,shape,_,controller_source,task_kind=next(iter(signatures))
    features={"observation.state":{"dtype":"float32","shape":(16,),"names":NAMES},
              "action":{"dtype":"float32","shape":(16,),"names":NAMES}}
    for cam in CAMERAS:
        features[f"observation.images.{cam}"]={"dtype":"video","shape":shape,
                                               "names":["height","width","channels"]}
    ds=LeRobotDataset.create(repo_id=repo_id,fps=fps,root=output,robot_type="r1pro_mujoco_sim",
        features=features,use_videos=True,vcodec="h264",video_backend="pyav",
        image_writer_threads=2,streaming_encoding=True)
    # Pinned LeRobot 0.4.4 exposes CRF on its encoder, not on create().
    # Set before the first frame starts the encoding workers.
    ds._streaming_encoder.crf=18
    sources=[]
    try:
        for path,report in chosen:
            with h5py.File(path,"r") as f:
                for i in range(report["frames"]):
                    frame={"observation.state":f["state"][i],"action":f["action"][i],
                           "task":str(f.attrs["task"])}
                    frame.update({f"observation.images.{c}":f[f"images/{c}"][i] for c in CAMERAS})
                    ds.add_frame(frame)
                ds.save_episode()
            sources.append({**report,"raw_sha256":raw_digest(path)})
        ds.finalize()
    finally:
        ds.stop_image_writer()
    (output/"source_manifest.json").write_text(json.dumps({"source":"simulation",
        "lerobot_version":"0.4.4","video_encoding":{"codec":"h264","crf":18,"width":shape[1],"height":shape[0]},"included_non_success":include_all,
        "controller_source":controller_source,"task_kind":task_kind,
        "human_demonstration":controller_source=="human_teleoperation",
        "required_task_checks":require_task_checks,"episodes":sources,
        "split_guidance":"Split by session_id before training; compute normalization on training episodes only"},indent=2))
    # Read images and state with the real loader, not just file existence checks.
    loaded=LeRobotDataset(repo_id=repo_id,root=output,video_backend="pyav")
    for index in (0,len(loaded)-1):
        row=loaded[index]
        assert row["action"].shape==(16,)
        for cam in CAMERAS:
            assert tuple(row[f"observation.images.{cam}"].shape)==(3,shape[0],shape[1])
    print(json.dumps({"output":str(output),"episodes":len(chosen),"frames":len(loaded),
                      "loaded_first_last_frames":True},indent=2))


if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("--raw",type=Path,required=True)
    p.add_argument("--output",type=Path)
    p.add_argument("--repo-id",default="local/r1pro-tray-sim")
    p.add_argument("--include-all",action="store_true")
    p.add_argument("--require-task-checks",action="store_true")
    a=p.parse_args()
    if a.output:export(a.raw,a.output,a.repo_id,a.include_all,a.require_task_checks)
    else:print(json.dumps([validate(x) for x in sorted(a.raw.glob("*.h5"))],indent=2))
