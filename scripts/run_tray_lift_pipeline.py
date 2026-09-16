"""Single entry point: plan, physics validation, synchronized capture, LeRobot, review."""
import argparse,json,subprocess,sys
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--scene',type=Path,default=Path('outputs/r1pro_tray_scene/scene_direct_grasp_720p.xml'));p.add_argument('--trajectory',type=Path);a=p.parse_args()
if a.output.exists():raise FileExistsError('Choose a new output directory: '+str(a.output))
a.output.mkdir(parents=True)
def run(script,*args):subprocess.run([sys.executable,'scripts/'+script,*map(str,args)],check=True)
trajectory=a.trajectory
if trajectory is None:
 run('generate_tray_lift.py','--scene',a.scene,'--output',a.output/'plan','--yaw',0,'--grasp-z',.87,'--approach-z',.98,'--approach-x',1.0,'--lift-x',1.0,'--lift-height',.15,'--pitch',-30)
 trajectory=a.output/'plan/trajectory.npz'
run('finalize_generated_tray_lift.py','--input',trajectory,'--scene',a.scene,'--output',a.output/'validated')
if not json.loads((a.output/'validated/report.json').read_text())['success']:raise RuntimeError('Physics lift acceptance failed; no success dataset exported')
run('generated_tray_to_lerobot.py','--trajectory',a.output/'validated/trajectory.npz','--scene',a.scene,'--output',a.output/'dataset','--repo-id','local/r1pro-direct-grasp-720p')
run('review_generated_lerobot.py','--run',a.output/'dataset','--scene',a.scene)
report=json.loads((a.output/'dataset/review_validation.json').read_text())
if not report['direct_grasp_passed'] or not report['replay_passed'] or not report['both_cameras_outward_during_dual_contact']:raise RuntimeError('Final replay/camera review failed')
print('SUCCESS: '+str(a.output/'dataset/lerobot'))
