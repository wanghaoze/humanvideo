import numpy as np
import mujoco
from r1pro_teleop.core import Simulation

def test_launch_pose_matches_original_torso_kinematics():
    s=Simulation('outputs/r1pro_tray_scene/scene_launch_pose.xml',render=False)
    try:
        original=mujoco.MjModel.from_xml_path('outputs/r1pro_tray_scene/scene.xml');d=mujoco.MjData(original)
        for i,q in enumerate([.2,-.5,-.72,0],1):d.qpos[original.jnt_qposadr[original.joint(f'torso_joint{i}').id]]=q
        mujoco.mj_forward(original,d)
        shift=s.data.body('base_link').xpos-d.body('base_link').xpos
        for i in range(1,5):
            np.testing.assert_allclose(s.data.body(f'torso_link{i}').xpos-shift,d.body(f'torso_link{i}').xpos,atol=1e-8)
            np.testing.assert_allclose(s.data.body(f'torso_link{i}').xmat,d.body(f'torso_link{i}').xmat,atol=1e-8)
        np.testing.assert_allclose(s.target[[3,11]],[-1.62,-1.62],atol=1e-6)
        np.testing.assert_allclose(s.target[[7,15]],[.1,.1],atol=1e-6)
        s.target[:]=0;s.reset()
        np.testing.assert_allclose(s.target[[3,11]],[-1.62,-1.62],atol=1e-6)
    finally:s.close()
