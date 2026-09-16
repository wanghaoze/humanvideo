"""Offline data review and conservative tray-task checks; never relabel demonstrations."""
import argparse
import json
from pathlib import Path
import h5py
import numpy as np


def task_checks(pose, state, fps, goal=(0.85, 0.32, 0.8)):
    goal = np.asarray(goal)
    xyz = pose[:, :3]
    q = pose[:, 3:7]  # MuJoCo wxyz
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 1]**2 + q[:, 2]**2), -1, 1)))
    tail = min(len(xyz), int(fps) + 1)
    speed = np.linalg.norm(np.diff(xyz, axis=0), axis=1) * fps
    checks = {
        'starts_away_from_goal': bool(np.linalg.norm(xyz[0, :2] - goal[:2]) > 0.1),
        'lifted_at_least_4cm': bool(xyz[:, 2].max() - xyz[0, 2] >= 0.04),
        'final_window_at_least_1s': len(xyz) >= fps + 1,
        'final_xy_within_4cm': bool(np.all(np.linalg.norm(xyz[-tail:, :2]-goal[:2], axis=1) <= 0.04)),
        'final_height_within_2cm': bool(np.all(abs(xyz[-tail:, 2]-goal[2]) <= 0.02)),
        'final_tilt_below_15deg': bool(np.all(tilt[-tail:] <= 15)),
        'final_speed_below_3cm_s': bool(np.all(speed[-max(1, tail-1):] <= 0.03)),
        'both_grippers_open': bool(np.all(state[-tail:, [7, 15]] >= 0.06)),
    }
    return {'checks': checks, 'candidate_success': all(checks.values()),
            'max_lift_m': float(xyz[:, 2].max()-xyz[0, 2]),
            'final_goal_distance_m': float(np.linalg.norm(xyz[-1, :2]-goal[:2])),
            'goal_bottom_center_m': goal.tolist(),
            'limitation': 'Kinematic checks only; does not prove grasp contact or expert demonstration quality.'}


def lift_checks(f,pose,fps):
    xyz=pose[:,:3];q=pose[:,3:];q=q/np.linalg.norm(q,axis=1,keepdims=True)
    tilt=np.degrees(np.arccos(np.clip(1-2*(q[:,1]**2+q[:,2]**2),-1,1)))
    tail=min(len(xyz),2*fps+1)
    checks={
        'hold_window_at_least_2s':len(xyz)>=2*fps+1,
        'held_above_initial_by_8cm':bool(np.all(xyz[-tail:,2]-xyz[0,2]>=.08)),
        'hold_tilt_below_10deg':bool(np.all(tilt[-tail:]<10)),
        'hold_speed_below_3cm_s':bool(np.all(np.linalg.norm(np.diff(xyz[-tail:],axis=0),axis=1)*fps<.03)),
        'left_contact_present':bool(np.all(f['contact/left_force'][-tail:]>0)),
        'right_contact_present':bool(np.all(f['contact/right_force'][-tail:]>0)),
        'held_without_table_contact':bool(np.all(f['contact/table_count'][-tail:]==0)),
        'no_other_robot_collision':bool(np.all(f['contact/other_count'][:]==0)),
        'left_wrist_faces_outward':bool(np.all(f['camera_check/left_outward'][-tail:]>0)),
        'right_wrist_faces_outward':bool(np.all(f['camera_check/right_outward'][-tail:]>0))}
    if f.attrs.get('grasp_style')=='direct_camera_front':
        for side in ('left','right'):
            dual=(f['contact/left_force'][:]>0)&(f['contact/right_force'][:]>0)
            checks[side+'_physical_camera_ahead_during_grasp']=bool(np.any(dual) and np.all(f['camera_check/'+side+'_ahead'][:][dual]>0))
            checks[side+'_wrist_unflipped_all_frames']=bool(np.all(f['camera_check/'+side+'_unflipped'][:]>0))
    return {'checks':checks,'candidate_success':all(checks.values()),'max_lift_m':float(xyz[:,2].max()-xyz[0,2]),'limitation':'Simulation lift only; not a transfer/place or real-robot demonstration.'}


def inspect(path):
    from .dataset import validate
    path = Path(path)
    report = {'file': str(path), 'valid': False, 'training_candidate': False}
    try:
        basic = validate(path)
        with h5py.File(path, 'r') as f:
            n, fps = basic['frames'], basic['fps']
            arrays = {k: f[k][:] for k in ('tray_pose', 'wall_monotonic', 'input_age_s', 'qpos', 'qvel')}
            for name, a in arrays.items():
                if len(a) != n or not np.isfinite(a).all():
                    raise ValueError(f'invalid {name}')
            pose, wall, age = (arrays[k] for k in ('tray_pose', 'wall_monotonic', 'input_age_s'))
            if pose.shape != (n, 7) or not np.allclose(np.linalg.norm(pose[:, 3:], axis=1), 1, atol=1e-3):
                raise ValueError('invalid tray quaternion/shape')
            if wall.shape != (n,) or age.shape != (n,) or np.any(np.diff(wall) <= 0):
                raise ValueError('invalid wall-clock/input age timeline')
            state, action = f['state'][:], f['action'][:]
            program_generated=f.attrs.get('controller_source')=='program_generated'
            task = lift_checks(f,pose,fps) if f.attrs.get('task_kind')=='tray_lift' else task_checks(pose, state, fps, json.loads(f.attrs.get('task_goal_xyz','[0.85,0.32,0.8]')))
            cameras = {}
            for name in json.loads(f.attrs['camera_names']):
                images = f['images/' + name]
                std = np.array([images[i].std() for i in range(n)])
                cameras[name] = {'blank_frame_fraction': float(np.mean(std < 2)),
                                 'minimum_pixel_std': float(std.min())}
            warnings = []
            stale = float(np.mean((age > 0.25) | (age < 0)))
            realtime_ratio = float((n - 1) / fps / (wall[-1] - wall[0]))
            jump = float(np.max(abs(np.diff(action[:, [0,1,2,3,4,5,6,8,9,10,11,12,13,14]], axis=0))))
            if stale > 0.1 and not program_generated: warnings.append('More than 10% of frames have stale or absent controller input')
            if realtime_ratio < 0.8 and not program_generated: warnings.append('Simulation ran substantially slower than wall time')
            if jump > 0.1: warnings.append('Arm target jumps exceed 0.1 rad/frame')
            blank = any(c['blank_frame_fraction'] > 0 for c in cameras.values())
            if blank: warnings.append('Blank/near-uniform camera frames detected')
            if basic['result'] == 'success' and not task['candidate_success']:
                warnings.append('Success label disagrees with tray task checks')
            report.update(basic, valid=True, task=task, cameras=cameras, warnings=warnings,
                          stale_or_absent_input_fraction=stale, realtime_ratio=realtime_ratio,
                          max_arm_target_step_rad=jump,
                          training_candidate=basic['result']=='success' and task['candidate_success'] and not warnings)
    except (ValueError, KeyError, OSError, TypeError, IndexError) as e:
        report['error'] = str(e)
    return report


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--raw', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    paths = sorted(args.raw.glob('*.h5'))
    episodes = [inspect(x) for x in paths if not x.name.endswith('.partial.h5')]
    report = {'episodes': episodes, 'completed': len(episodes),
              'incomplete_files': [str(x) for x in paths if x.name.endswith('.partial.h5')],
              'valid': sum(x['valid'] for x in episodes),
              'training_candidates': sum(x['training_candidate'] for x in episodes),
              'note': 'Review report only; source labels/files remain unchanged. Goal thresholds are for the supplied scene.'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2))
    print(json.dumps({k:v for k,v in report.items() if k != 'episodes'}, indent=2))


if __name__ == '__main__': main()
