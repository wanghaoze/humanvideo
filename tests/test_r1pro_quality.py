import numpy as np
from r1pro_teleop.quality import task_checks, inspect


def trajectory():
    p=np.zeros((80,7));p[:,0]=.85;p[:,2]=.8;p[:,3]=1
    p[:40,1]=np.linspace(0,.32,40);p[40:,1]=.32
    p[5:35,2]+=.08*np.sin(np.linspace(0,np.pi,30))
    s=np.zeros((80,16));s[:,[7,15]]=.08
    return p,s


def test_kinematic_candidate_is_not_a_success_label():
    p,s=trajectory()
    assert task_checks(p,s,20)['candidate_success']


def test_sliding_tray_to_goal_is_not_lift_and_place():
    p,s=trajectory();p[:,2]=.8
    assert not task_checks(p,s,20)['candidate_success']


def test_holding_tray_and_unstable_placement_rejected():
    p,s=trajectory();s[-10:,7]=.02
    assert not task_checks(p,s,20)['candidate_success']
    p,s=trajectory();p[-5:,1]+=.03
    assert not task_checks(p,s,20)['candidate_success']


def test_invalid_record_reported_without_crashing(tmp_path):
    p=tmp_path/'broken.h5';p.write_bytes(b'not hdf5')
    r=inspect(p)
    assert not r['valid'] and not r['training_candidate'] and r['error']
