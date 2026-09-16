"""Regression: a flipped physical wrist must fail even if its virtual camera looks out."""
import h5py
import numpy as np
from r1pro_teleop.quality import lift_checks
from r1pro_teleop.core import Simulation


def test_direct_grasp_checks_physical_camera(tmp_path):
    n = 100
    pose = np.zeros((n, 7)); pose[:, 3] = 1
    pose[1:, 2] = .12
    with h5py.File(tmp_path / 'episode.h5', 'w') as f:
        f.attrs['grasp_style'] = 'direct_camera_front'
        for side in ('left', 'right'):
            f['contact/' + side + '_force'] = np.ones(n)
            f['camera_check/' + side + '_outward'] = np.ones(n)
            f['camera_check/' + side + '_ahead'] = np.full(n, .02)
            f['camera_check/' + side + '_unflipped'] = np.ones(n)
        f['contact/table_count'] = np.zeros(n)
        f['contact/other_count'] = np.zeros(n)
        assert lift_checks(f, pose, 20)['candidate_success']
        f['camera_check/left_ahead'][50] = -.02
        assert not lift_checks(f, pose, 20)['candidate_success']
        f['camera_check/left_ahead'][50] = .02
        f['camera_check/right_unflipped'][10] = -1
        assert not lift_checks(f, pose, 20)['candidate_success']


def test_direct_scene_preserves_home_and_supports_native_720p():
    old = Simulation('outputs/r1pro_tray_scene/scene_launch_pose.xml', render=False)
    new = Simulation('outputs/r1pro_tray_scene/scene_direct_grasp_720p.xml', render=False)
    try:
        np.testing.assert_array_equal(old.data.qpos, new.data.qpos)
        np.testing.assert_array_equal(old.model.body_mass, new.model.body_mass)
        np.testing.assert_array_equal(old.model.geom_friction, new.model.geom_friction)
        assert new.model.vis.global_.offwidth >= 1280
        assert new.model.vis.global_.offheight >= 720
    finally:
        old.close(); new.close()
