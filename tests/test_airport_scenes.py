import copy
import json
from pathlib import Path
import mujoco
import numpy as np
import pytest
from r1pro_teleop.airport_scenes import (
    TEMPLATE, sample, slots, validate_spec, build_xml, physics_check, overlap,
)


def fixture():
    return sample(20260920, 4, 0, 'ordinary', 'smoke', 0)


def test_seed_and_slot_formula():
    assert fixture() == fixture()
    s = fixture(); validate_spec(s)
    x = [slot['position_belt'][0] for slot in slots()]
    np.testing.assert_allclose(np.diff(x), .43 + .015)
    assert all(slot['yaw_belt'] == np.pi / 2 for slot in slots())
    assert sample(20260921, 4, 0, 'ordinary', 'smoke', 0) != s


def test_reject_unsupported_overlap_and_bad_slot():
    s = fixture(); s['trays'][0]['position_world'][0] = 8
    with pytest.raises(ValueError, match='com_outside_table'): validate_spec(s)
    s = fixture(); s['trays'][1]['position_world'] = s['trays'][0]['position_world'][:]
    with pytest.raises(ValueError, match='ordinary_overlap'): validate_spec(s)
    s = fixture(); s['slots'][0]['position_belt'][0] = 0
    with pytest.raises(ValueError, match='slot_outside_belt'): validate_spec(s)


def test_physics_and_preserved_configuration(tmp_path):
    s = fixture(); p = tmp_path / 'scene.xml'
    xml = build_xml(s, TEMPLATE.resolve(), p); p.write_text(xml)
    assert xml == build_xml(copy.deepcopy(s), TEMPLATE.resolve(), p)
    check = physics_check(xml, p, s)
    assert check['passed'] and check['max_tilt_deg'] < 10
    baseline = mujoco.MjModel.from_xml_path(str(TEMPLATE))
    model = mujoco.MjModel.from_xml_path(str(p))
    np.testing.assert_allclose(model.key_qpos[0, :18], baseline.key_qpos[0, :18])
    np.testing.assert_array_equal(model.actuator_gainprm, baseline.actuator_gainprm)
    np.testing.assert_array_equal(model.actuator_forcerange, baseline.actuator_forcerange)
    assert model.vis.global_.offwidth == 1280 and model.vis.global_.offheight == 720
    for side in ('left', 'right'):
        np.testing.assert_allclose(model.camera(side+'_wrist').quat, baseline.camera(side+'_wrist').quat)
        np.testing.assert_allclose(model.camera(side+'_wrist').pos, baseline.camera(side+'_wrist').pos)
    for i in range(3):
        assert model.body(f'tray_{i}').mass == pytest.approx(baseline.body('tray').mass)
    # A tray deliberately embedded in the tabletop must not pass the physical gate.
    s['trays'][0]['position_world'][2] = .78
    xml = build_xml(s, TEMPLATE.resolve(), p); p.write_text(xml)
    with pytest.raises(ValueError, match='illegal_penetration|slid_off'): physics_check(xml, p, s)


def test_real_robot_collision_rejected(tmp_path):
    s = fixture(); s['robot']['base_position_world'] = [.85, 0, 0]
    p = tmp_path/'collision.xml'; xml = build_xml(s, TEMPLATE.resolve(), p); p.write_text(xml)
    with pytest.raises(ValueError, match='robot_initial_collision'): physics_check(xml, p, s)
