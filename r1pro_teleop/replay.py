"""Replay saved actions through physics, compare recorded states, and export a review video."""
import argparse
import hashlib
import json
from pathlib import Path
import h5py
import mujoco
import numpy as np
from .core import Simulation
from .dataset import validate


def run(raw, scene, output, video=True):
    basic = validate(raw)
    if hashlib.sha256(scene.read_bytes()).hexdigest() != basic['scene_sha256']:
        raise ValueError('Scene XML differs from recording; use the matching scene')
    output.mkdir(parents=True, exist_ok=False)
    sim = Simulation(scene, fps=basic['fps'], render=video)
    container = None
    try:
        if video:
            import av
            container = av.open(str(output/'replay.mp4'), 'w')
            stream = container.add_stream('libx264', rate=basic['fps'])
            stream.width, stream.height, stream.pix_fmt = 320, 240, 'yuv420p'
        errors, tray_errors = [], []
        with h5py.File(raw, 'r') as f:
            sim.data.qpos[:] = f['qpos'][0]
            sim.data.qvel[:] = f['qvel'][0]
            if 'qacc_warmstart' in f: sim.data.qacc_warmstart[:] = f['qacc_warmstart'][0]
            mujoco.mj_forward(sim.model, sim.data)
            for i in range(basic['frames']):
                errors.append(abs(sim.state() - f['state'][i]))
                tray_errors.append(np.linalg.norm(sim.data.body('tray').xpos-f['tray_pose'][i,:3]))
                if video:
                    frame = av.VideoFrame.from_ndarray(sim.image(), format='rgb24')
                    for packet in stream.encode(frame): container.mux(packet)
                sim.target[:] = f['action'][i]
                sim.step()
        if container:
            for packet in stream.encode(): container.mux(packet)
        errors = np.asarray(errors)
        arm = float(errors[:, [0,1,2,3,4,5,6,8,9,10,11,12,13,14]].max())
        grip, tray = float(errors[:, [7,15]].max()), float(max(tray_errors))
        report = {'raw': str(raw), 'frames': basic['frames'], 'fps': basic['fps'],
                  'max_arm_error_rad': arm, 'max_gripper_error_m': grip,
                  'max_tray_position_error_m': tray,
                  'within_tolerance': arm < .02 and grip < .003 and tray < .01,
                  'video': str(output/'replay.mp4') if video else None,
                  'note': 'Action replay consistency, not a successful-grasp test. Older recordings omit solver warm-start state.'}
        (output/'replay_report.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
        return report
    finally:
        if container: container.close()
        sim.close()


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--raw', type=Path, required=True)
    p.add_argument('--scene', type=Path, default=Path('outputs/r1pro_tray_scene/scene_physics.xml'))
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--no-video', action='store_true')
    a = p.parse_args()
    if not run(a.raw, a.scene, a.output, not a.no_video)['within_tolerance']: raise SystemExit(2)
