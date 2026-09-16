"""No-headset acceptance checks: physics, contact cavity, rendering and recording."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from PIL import Image
from .core import Simulation,Recorder,CAMERAS
from .dataset import validate


def check_cavity(scene):
    tree=ET.parse(scene);root=tree.getroot()
    # Resolve assets for the in-memory model used by this test.
    for mesh in root.findall("./asset/mesh"):
        mesh.set("file",str((scene.parent/mesh.get("file")).resolve()))
    b=ET.SubElement(root.find("worldbody"),"body",name="test_probe",pos="0.85 0 0.861")
    ET.SubElement(b,"freejoint",name="probe_free")
    ET.SubElement(b,"geom",name="probe",type="sphere",size="0.004",mass="0.01")
    m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding="unicode"));d=mujoco.MjData(m)
    probe=m.geom("probe").id
    mujoco.mj_forward(m,d)
    def hits():
        return [c for c in d.contact if probe in (c.geom1,c.geom2) and
                (m.geom(c.geom1).name.startswith("tray_contact") or m.geom(c.geom2).name.startswith("tray_contact"))]
    assert not hits(),"Tray cavity is incorrectly blocked by convex collisions"
    adr=m.jnt_qposadr[m.joint("probe_free").id]
    d.qpos[adr:adr+3]=[0.85+0.273,0,0.801+0.095]
    mujoco.mj_forward(m,d)
    assert hits(),"Probe does not contact the tray rim/wall"
    return {"cavity_open":True,"rim_contact":True}


def run(scene,output,frames=40):
    scene=Path(scene).resolve();output=Path(output);output.mkdir(parents=True,exist_ok=True)
    sim=Simulation(scene)
    report={"source":"simulation","nq":sim.model.nq,"nu":sim.model.nu,"ncam":sim.model.ncam}
    try:
        assert sim.model.nu==16 and sim.model.ncam==4
        assert sim.model.body_jntnum[sim.model.body("tray").id]==1
        report.update(check_cavity(scene))
        z0=float(sim.data.body("tray").xpos[2])
        for _ in range(100):sim.step()
        z=float(sim.data.body("tray").xpos[2]);q=sim.data.qpos.copy()
        assert 0.78<z<0.84,f"Tray has fallen through table or floats: {z}"
        assert np.linalg.norm(sim.data.qvel)<2,f"Unstable resting model: {np.linalg.norm(sim.data.qvel)}"
        report.update(tray_initial_z=z0,tray_settled_z=z,rest_velocity_norm=float(np.linalg.norm(sim.data.qvel)))
        images=sim.images()
        for name,img in images.items():
            assert img.std()>2,f"Empty image {name}"
            Image.fromarray(img).save(output/(name+".png"))
        Image.fromarray(sim.image()).save(output/"overview.png")
        rec=Recorder(output/"raw",sim,task="Pipeline smoke test; not a successful tray demonstration")
        try:
            for i in range(frames):
                sim.target[6]=0.03*np.sin(i/10)
                rec.append(sim,sim.images(),-1.)
                sim.step()
        except Exception:
            rec.finish("failure");raise
        raw=rec.finish("smoke")
        report["raw"]=validate(raw)
        assert report["raw"]["max_state_action_difference"]>1e-5,"Actions were incorrectly copied from state"
        report["note"]="Validates load/physics/rest/cavity/images/data only; no claim of successful grasp or Quest hardware test"
        (output/"smoke_report.json").write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2))
    finally:sim.close()


if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--scene",type=Path,default=Path("outputs/r1pro_tray_scene/scene_physics.xml"))
    p.add_argument("--output",type=Path,default=Path("runs/r1pro_smoke"));p.add_argument("--frames",type=int,default=40)
    a=p.parse_args();run(a.scene,a.output,a.frames)
