import pytest

from r1pro_teleop.quest_bridge import parse_line


IDENTITY = "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"
POSE = "1 0 0 0.2 0 1 0 1.1 0 0 1 -0.4 0 0 0 1"


def test_beta_opposite_hand_tracking_does_not_validate_pose():
    with pytest.raises(ValueError, match="no valid controller poses"):
        parse_line(f"l:{POSE}&R,rightGrip 1")


def test_identity_placeholder_is_rejected_even_with_tracking_flag():
    with pytest.raises(ValueError, match="no valid controller poses"):
        parse_line(f"l:{IDENTITY}&L")


def test_valid_hand_survives_other_hand_placeholder():
    packet = parse_line(f"l:{IDENTITY}|r:{POSE}&L,R,rightGrip 0.7")
    assert set(packet["hands"]) == {"right"}
    assert packet["hands"]["right"]["grip"] == 0.7
