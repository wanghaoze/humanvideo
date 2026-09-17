from pathlib import Path
import numpy as np
import mujoco
import pytest
from r1pro_teleop.airport_mobile import make_mobile_scene,move_base,collision_pairs
from r1pro_teleop.airport_recording import AirportSimulation

SOURCE=Path('outputs/airport_trays_indoor_v1_1/smoke/ordinary/airport_0001_20260917/scene.xml')

def test_mobile_servo_and_legacy_dimensions(tmp_path):
    xml=tmp_path/'scene.xml';make_mobile_scene(SOURCE,xml)
    fixed=AirportSimulation(SOURCE,render=False);s=AirportSimulation(xml,render=False)
    try:
        assert s.base_control_available and not fixed.base_control_available
        assert s.state().shape==(16,)
        assert np.allclose(s.state(),fixed.state())
        assert np.allclose(s.data.cam_xpos[s.model.camera('head').id],fixed.data.cam_xpos[fixed.model.camera('head').id])
        origin=s.base_state().copy();goal=origin+np.array([0,-.1,.12])
        info=move_base(s,goal)
        assert info['position_error_m']<.002
        assert info['yaw_error_rad']<.002
        assert not collision_pairs(s)
        assert np.linalg.norm(s.base_joint_state())>.1
        assert np.allclose(s.target,fixed.target)
        s.reset();assert np.allclose(s.base_state(),origin)
        with pytest.raises(FileExistsError):make_mobile_scene(SOURCE,xml)
    finally:s.close();fixed.close()
