import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from r1pro_teleop.core import Simulation

@pytest.fixture
def sim():
    s=Simulation("outputs/r1pro_tray_scene/scene_physics.xml",render=False)
    yield s
    s.close()

def send(s,seq,p=(0,1,0),angle=0,grip=.8):
    s.input({"seq":seq,"hands":{"left":{"position":p,
        "quaternion_xyzw":Rotation.from_rotvec([0,angle,0]).as_quat().tolist(),
        "grip":grip,"trigger":0}},"buttons":{}},now=seq*.05+1)
    s.update_control(now=seq*.05+1)

def test_noise_deadband_gain_and_grip_hysteresis(sim):
    goals=[]
    sim.ik=lambda side,p,r: (goals.append((p.copy(),r.copy())) or sim.target[:7].copy())
    send(sim,0);initial=goals[-1][0].copy()
    for i in range(1,20):send(sim,i,p=(.0008*(-1)**i,1,0),angle=.003*(-1)**i,grip=.48)
    assert np.max(np.abs(np.array([g[0] for g in goals])-initial))<1e-8
    for i in range(20,40):send(sim,i,p=(.1,1,0),angle=.3)
    assert abs(np.linalg.norm(goals[-1][0]-initial)-.15)<.003
    angle=Rotation.from_matrix(goals[-1][1]@goals[0][1].T).magnitude()
    assert abs(angle-.39)<.015
    send(sim,40,grip=.2)
    assert not sim.anchors and not sim.joint_velocity
    sim.update_control(now=4)
    assert not sim.anchors

def test_real_ik_rotation_and_position_following(sim):
    for _ in range(50):sim.step()
    p0,r0=sim.tcp('left')
    send(sim,0)
    for i in range(1,61):
        send(sim,i,p=(.05,1,0),angle=.2)
        sim.step()
    p1,r1=sim.tcp('left')
    assert np.linalg.norm(p1-(p0+[0,-.075,0]))<.025
    assert Rotation.from_matrix(r1@r0.T).magnitude()>.18
    old=sim.target.copy()
    sim.update_control(now=10)
    assert np.array_equal(old,sim.target)
