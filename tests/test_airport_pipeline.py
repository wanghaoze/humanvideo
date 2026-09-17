import json
from pathlib import Path
from collections import Counter
import numpy as np
import pytest
from r1pro_teleop.airport_annotations import annotate_arrays,angle_error
from r1pro_teleop.airport_pipeline import split_map
from r1pro_teleop.airport_scenes import sample

def fixture(n=30):
 spec=sample(20260919,3,3,'ordinary','train',0)
 xyz=np.array([[.55+s['position_belt'][1],.6-s['position_belt'][0],.8] for s in spec['slots']])
 pose=np.zeros((n,3,7));pose[:,:,:3]=xyz;pose[:,:,3]=1
 a=dict(pose=pose,forces=np.zeros((n,3,2)),table=np.ones((n,3)),support=np.ones((n,3)),floor=np.zeros((n,3)),bottom=np.full((n,3),.8),illegal=np.zeros(n),penetration=np.zeros(n),state=np.tile([0]*7+[.1]+[0]*7+[.1],(n,1)),tcp=np.tile([[0,0,1],[0,.5,1]],(n,1,1)))
 return a,spec

def test_stable_placement_wrap_and_gap():
 a,s=fixture();r=annotate_arrays(a,s,'test','program_generated');assert r['success']
 assert any(e['event']=='task_complete' and e['frame']==20 for e in r['events'])
 assert abs(angle_error(np.pi-.01,-np.pi+.01)+.02)<1e-9
 a,s=fixture(20);assert not annotate_arrays(a,s,'test','program_generated')['success']
 a,s=fixture();a['pose'][:,1,1]+=.02 # overlap even though position error remains <3cm
 assert not annotate_arrays(a,s,'test','program_generated')['success']
 a,s=fixture();a['floor'][5,0]=1
 assert not annotate_arrays(a,s,'test','program_generated')['success']

def test_unknown_abort_and_half_open_segments():
 a,s=fixture();a['pose'][:,:,0]+=1
 r=annotate_arrays(a,s,'test','human_teleoperation','aborted')
 assert r['result']=='aborted' and r['events'][-1]['event']=='manual_abort'
 assert r['segments'][0]['needs_review'] and r['segments'][0]['target_tray_id'] is None
 segments=r['segments'];assert segments[0]['start_frame']==0 and segments[-1]['end_frame']==30
 for x,y in zip(segments,segments[1:]):assert x['end_frame']==y['start_frame']

def test_group_splits():
 for n,counts in [(12,(8,2,2)),(400,(300,50,50))]:
  ids=[f'scene_{i}' for i in range(n)];mapping=split_map(ids,counts)
  assert mapping==split_map(list(reversed(ids)),counts)
  assert Counter(mapping.values())==dict(zip(['train','val','test'],counts))

def test_real_physical_fixture_events():
 p=Path('runs/airport_phases_fixture_v2/report.json')
 if not p.exists():pytest.skip('Run the real MuJoCo lift/place fixture first')
 r=json.loads(p.read_text());assert r['passed']
 frames={x['event']:x['frame'] for x in r['events']}
 assert frames['gripper_contact']<frames['lift_above_2cm']<frames['table_support_restored']<frames['open_release']


def test_observed_base_motion_has_no_guessed_target():
 a,s=fixture();s['base_control_available']=True;a['pose'][:,:,0]+=2
 a['base_pose']=np.column_stack([np.linspace(0,.1,30),np.zeros(30),np.zeros(30)])
 r=annotate_arrays(a,s,'base','program_generated')
 assert r['base_control_available']
 assert any(e['event']=='base_motion_started' for e in r['events'])
 moving=[x for x in r['segments'] if x['skill']=='navigate_to_tray']
 assert moving and all(x['target_tray_id'] is None and x['needs_review'] for x in moving)


def test_rectangular_axis_contract_is_explicit():
 a,s=fixture();a['pose'][:,:,3:]=[0,0,0,1]
 assert not annotate_arrays(a,s,'axis','program_generated')['success']
 s['yaw_period_rad']=float(np.pi)
 assert annotate_arrays(a,s,'axis','program_generated')['success']
 a['pose'][:,:,3:]=[np.sqrt(.5),0,0,np.sqrt(.5)]
 assert not annotate_arrays(a,s,'axis','program_generated')['success']
