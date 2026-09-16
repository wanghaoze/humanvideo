from pathlib import Path
import numpy as np
from r1pro_teleop.core import Simulation


def test_home_forearms_stay_horizontal_with_open_grippers():
    root=Path(__file__).resolve().parents[1]
    sim=Simulation(root/'outputs/r1pro_tray_scene/scene_physics.xml',render=False)
    try:
        for _ in range(100):sim.step()
        for side,opening_index in [('left',7),('right',15)]:
            axis=sim.data.body(side+'_arm_link6').xpos-sim.data.body(side+'_arm_link4').xpos
            tilt=np.degrees(np.arctan2(abs(axis[2]),np.linalg.norm(axis[:2])))
            assert tilt<1.0
            assert sim.state()[opening_index]>0.095
        for contact in sim.data.contact:
            for geom in (contact.geom1,contact.geom2):
                body=sim.model.body(int(sim.model.geom_bodyid[geom])).name
                assert not body.startswith(('left_arm','right_arm','left_gripper','right_gripper'))
    finally:sim.close()
