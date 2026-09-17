"""Real MuJoCo lift+lower+release fixture using the existing validated single-tray plan."""
import argparse,json,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from r1pro_teleop.core import Simulation
from r1pro_teleop.airport_recording import Evidence
from r1pro_teleop.airport_annotations import annotate_arrays
from r1pro_teleop.airport_pipeline import save

def main():
 p=argparse.ArgumentParser();p.add_argument('--trajectory',type=Path,required=True);p.add_argument('--scene',type=Path,default=Path('outputs/r1pro_tray_scene/scene_direct_grasp_720p.xml'));p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
 data=np.load(a.trajectory);actions=list(data['action']);lift=data['action'][data['stage']=='lift'];actions.extend(lift[::-1]);last=actions[-1].copy()
 for i in range(40):
  row=last.copy();row[[7,15]]=last[[7,15]]+(.1-last[[7,15]])*(i+1)/40;actions.append(row)
 actions.extend([actions[-1].copy() for _ in range(60)])
 sim=Simulation(a.scene,render=False);evidence=Evidence(sim,['tray']);rows=[];states=[]
 try:
  for action in actions:
   sim.target=np.asarray(action,np.float32);rows.append(evidence.row());states.append(sim.state());sim.step()
 finally:sim.close()
 keys={'pose':'tray_pose','forces':'gripper_contact_force','table':'table_contact','support':'support_contact','floor':'floor_contact','bottom':'min_z','illegal':'illegal_collision','penetration':'max_penetration','tcp':'tcp'}
 arrays={k:np.array([r['privileged/'+v] for r in rows]) for k,v in keys.items()};arrays['state']=np.array(states)
 ann=annotate_arrays(arrays,{'scene_id':'single_tray_physical_fixture'},'physical_lift_place_fixture','program_generated',fps=20)
 events=[x['event'] for x in ann['events']]
 report={'source':'program_generated','environment':'simulation','fixture_only':True,'frames':len(rows),'events':ann['events'],'lift_detected':'lift_above_2cm' in events,'support_restored':'table_support_restored' in events,'open_release_detected':'open_release' in events,'last_table_contact':arrays['table'][-1].tolist(),'last_hand_forces':arrays['forces'][-1].tolist(),'floor_contact_frames':int(np.any(arrays['floor']>0,axis=1).sum())}
 report['passed']=all(report[k] for k in ('lift_detected','support_restored','open_release_detected')) and report['floor_contact_frames']==0
 save(a.output/'report.json',report);save(a.output/'segments.json',ann['segments']);np.savez_compressed(a.output/'truth.npz',**arrays);print(json.dumps(report,indent=2))
 if not report['passed']:raise RuntimeError('Physical boundary fixture failed')
if __name__=='__main__':main()
