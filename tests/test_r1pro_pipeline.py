import copy
import numpy as np
import pytest
from fastapi.testclient import TestClient
from r1pro_teleop.core import validate_packet
from r1pro_teleop.quest_bridge import parse_line
from r1pro_teleop.server import Service,create_app


def packet():
    return {"seq":0,"client_id":"test-1","hands":{"left":{
        "position":[0,1,0],"quaternion_xyzw":[0,0,0,1],"grip":0.8,"trigger":0.4}},"buttons":{}}


def test_adb_wire_pose_and_analog_buttons():
    matrix="1 0 0 0.1 0 1 0 1.2 0 0 1 -0.3 0 0 0 1"
    x=parse_line(f"l:{matrix}|r:{matrix}&L,R,X,leftGrip 0.75,leftTrig 0.2,rightGrip 1,rightTrig 0.8")
    assert x["hands"]["left"]["position"]==[0.1,1.2,-0.3]
    assert x["hands"]["left"]["trigger"]==0.2
    assert x["buttons"]["X"] and not x["buttons"]["A"]


@pytest.mark.parametrize("field,value",[("position",[float('nan'),0,0]),("quaternion_xyzw",[0,0,0,0]),("grip",2)])
def test_invalid_pose_rejected(field,value):
    x=packet();x["hands"]["left"][field]=value
    with pytest.raises(ValueError):validate_packet(x)


def test_auth_replay_and_single_controller():
    s=Service("unused.xml","unused")
    # No lifespan context: these HTTP validation tests never create a renderer.
    c=TestClient(create_app(s,"test-token"))
    assert c.post("/api/input",json=packet()).status_code==401
    h={"Authorization":"Bearer test-token"}
    assert c.post("/api/input",json=packet(),headers=h).status_code==200
    assert c.post("/api/input",json=packet(),headers=h).status_code==409
    x=packet();x["seq"]=1;x["client_id"]="competing"
    assert c.post("/api/input",json=x,headers=h).status_code==409
    x=packet();x["seq"]=2;x["hands"]["left"]["trigger"]=-1
    assert c.post("/api/input",json=x,headers=h).status_code==422
