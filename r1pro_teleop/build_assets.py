"""Build compound convex collisions and a dynamic MJCF, preserving source assets."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
import coacd
import mujoco

ROOT = Path(__file__).resolve().parents[1]


def fast_tray_parts(dest):
    """Open rounded frustum, fitted to asset sections, without embossed lettering."""
    def ring(hx,hy,r):
        points=[]
        for cx,cy,angle in [(hx-r,hy-r,0),(-hx+r,hy-r,90),(-hx+r,-hy+r,180),(hx-r,-hy+r,270)]:
            for a in np.linspace(angle,angle+90,4):
                t=np.deg2rad(a);points.append([cx+r*np.cos(t),cy+r*np.sin(t)])
        return np.asarray(points)
    outer0=ring(.2632,.19753,.025)
    outer1=ring(.280,.215,.0268)
    inner0=ring(.2532,.18753,.017)
    inner1=ring(.270,.205,.0188)
    def at(x,z):return np.c_[x,np.full(len(x),z)]
    solids=[trimesh.convex.convex_hull(np.r_[at(outer0,0),at(outer0,.012)])]
    for i in range(len(outer0)):
        j=(i+1)%len(outer0)
        vertices=np.r_[at(outer0[[i,j]],.008),at(outer1[[i,j]],.105),
                       at(inner0[[i,j]],.012),at(inner1[[i,j]],.105)]
        solids.append(trimesh.convex.convex_hull(vertices))
    parts=[]
    for i,mesh in enumerate(solids):
        name=f"tray_contact_{i:03d}";mesh.export(dest/(name+".obj"))
        parts.append({"name":name,"file":f"assets/collision/{name}.obj",
                      "volume_m3":abs(float(mesh.volume)),"bounds_m":mesh.bounds.tolist()})
    return parts


def build(scene_dir, rebuild=False):
    scene_dir = Path(scene_dir).resolve()
    dest = scene_dir / "assets/collision"
    dest.mkdir(parents=True, exist_ok=True)
    manifest_file = dest / "manifest.json"
    if manifest_file.exists() and not rebuild:
        manifest = json.loads(manifest_file.read_text())
    else:
        manifest = {"method": "CoACD tray; connected-component convex hulls table",
                    "units": "m", "coacd": "1.0.14", "parts": {}, "sources": {}}
        for kind in ("tray", "table"):
            entries = []
            paths = [scene_dir / "assets/tray/tray_1.obj"] if kind == "tray" else sorted((scene_dir / "assets/table").glob("table_*.obj"))
            for path in paths:
                print("COLLISION", path.name, flush=True)
                manifest["sources"][str(path.relative_to(scene_dir))] = hashlib.sha256(path.read_bytes()).hexdigest()
                mesh = trimesh.load(path, force="mesh", process=True)
                if kind == "tray":
                    # The hull of the entire tray would fill its cavity. Decompose it.
                    coacd.set_log_level("warn")
                    results = coacd.run_coacd(coacd.Mesh(mesh.vertices, mesh.faces),
                        threshold=0.02, max_convex_hull=64, preprocess_mode="auto",
                        resolution=2000, mcts_nodes=20, mcts_iterations=100,
                        mcts_max_depth=3, seed=42)
                    pieces = [trimesh.Trimesh(v, f, process=True) for v, f in results]
                else:
                    # Separate beams/legs/bolts before taking hulls, preserving gaps.
                    pieces = [p.convex_hull for p in mesh.split(only_watertight=False)
                              if len(p.faces) >= 4 and np.min(p.extents) > 0.0001]
                for piece in pieces:
                    name = f"{kind}_collision_{len(entries):03d}"
                    piece.export(dest / (name + ".obj"))
                    entries.append({"name": name, "file": f"assets/collision/{name}.obj",
                                    "volume_m3": abs(float(piece.volume)),
                                    "bounds_m": piece.bounds.tolist()})
            if not entries:
                raise RuntimeError(f"No collision parts for {kind}")
            manifest["parts"][kind] = entries
        manifest_file.write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    if manifest.get("table_collision_revision") != 2:
        entries=[]
        for path in sorted((scene_dir/"assets/table").glob("table_*.obj")):
            mesh=trimesh.load(path,force="mesh",process=True)
            # OBJ normals/UV seams must not split a solid into isolated flat faces.
            mesh.merge_vertices(merge_tex=True,merge_norm=True)
            pieces=[mesh.convex_hull] if path.stem=="table_1" else [
                p.convex_hull for p in mesh.split(only_watertight=False)
                if len(p.faces)>=4 and np.min(p.extents)>0.0001]
            print("TABLE",path.name,len(pieces),flush=True)
            for piece in pieces:
                name=f"table_collision_v2_{len(entries):03d}"
                piece.export(dest/(name+".obj"))
                entries.append({"name":name,"file":f"assets/collision/{name}.obj",
                                "volume_m3":abs(float(piece.volume)),"bounds_m":piece.bounds.tolist()})
        manifest["parts"]["table"]=entries
        manifest["table_collision_revision"]=2
        manifest_file.write_text(json.dumps(manifest,indent=2),encoding="utf-8")

    manifest["runtime_tray_parts"]=fast_tray_parts(dest)
    manifest["runtime_tray_method"]="17 convex pieces: fitted rounded frustum and 12 mm base; source section approximation, no embossing; physical dimensions unmeasured"
    manifest_file.write_text(json.dumps(manifest,indent=2),encoding="utf-8")

    tree = ET.parse(scene_dir / "scene.xml")
    root = tree.getroot()
    root.set("model", "R1Pro_Tray_Quest_Simulation")
    ET.SubElement(root, "option", timestep="0.002", integrator="implicitfast",
                  gravity="0 0 -9.81", cone="elliptic", iterations="80")
    asset = root.find("asset")
    world = root.find("worldbody")
    # User-selected initial placement: 0.5 m toward the table along world +X.
    world.find("body[@name='base_link']").set("pos", "0.5 0 0")
    # Torso/chassis are fixed for this task. Do not train an action for them.
    for body in world.iter("body"):
        for j in list(body.findall("joint")):
            if j.get("name", "").startswith("torso_"):
                body.remove(j)
            else:
                j.set("damping", "2" if j.get("type") != "slide" else "10")
                j.set("armature", "0.02" if j.get("type") != "slide" else "0.005")
        if body.find("joint") is not None and "arm_link" in body.get("name", ""):
            body.set("gravcomp", "1")
    for key in ("table", "tray"):
        body = world.find(f"body[@name='{key}']")
        for geom in list(body.findall("geom")):
            if geom.get("contype") != "0":
                body.remove(geom)
            else:
                geom.set("mass", "0")
        if key == "tray":
            ET.SubElement(body, "freejoint", name="tray_free")
        pieces = manifest["runtime_tray_parts"] if key=="tray" else manifest["parts"][key]
        total_volume = sum(p["volume_m3"] for p in pieces)
        for part in pieces:
            ET.SubElement(asset, "mesh", name=part["name"], file=part["file"])
            attrs = dict(name=part["name"], type="mesh", mesh=part["name"],
                         group="3", friction="0.8 0.005 0.0001", condim="4",
                         solref="0.006 1", solimp="0.95 0.99 0.001")
            if key == "tray":
                attrs["mass"] = str(1.2 * part["volume_m3"] / total_volume)
            ET.SubElement(body, "geom", **attrs)
    actuator = ET.SubElement(root, "actuator")
    equality = ET.SubElement(root, "equality")
    for side in ("left", "right"):
        for i in range(1, 8):
            name = f"{side}_arm_joint{i}"
            j = root.find(f".//joint[@name='{name}']")
            limit = j.get("actuatorfrcrange", "-18 18")
            ET.SubElement(actuator, "position", name=name+"_target", joint=name,
                          kp="250", kv="25", ctrlrange=j.get("range"),
                          forcerange=limit)
        finger1 = f"{side}_gripper_finger_joint1"
        finger2 = f"{side}_gripper_finger_joint2"
        for name in (finger1, finger2):
            root.find(f".//joint[@name='{name}']").set("range", "0.002 0.05")
        ET.SubElement(equality, "joint", joint1=finger1, joint2=finger2,
                      polycoef="0 1 0 0 0", solref="0.004 1")
        ET.SubElement(actuator, "position", name=f"{side}_gripper_target", joint=finger1,
                      kp="2000", kv="40", ctrlrange="0.002 0.05", forcerange="-40 40")
        body = world.find(f".//body[@name='{side}_gripper_link']")
        ET.SubElement(body, "site", name=f"{side}_tcp", pos="0 0 -0.045",
                      size="0.006", rgba="0 0.8 0.5 1", group="4")
        # Wrist monitor offset above the fingers, looking forward/down at contact.
        wrist_view = {'left': {'pos': '0.18000000 0.00000000 0.00500000', 'xyaxes': '0.00000000 -1.00000000 0.00000000 0.67267279 0.00000000 -0.73994007'}, 'right': {'pos': '0.18000000 0.00000000 0.00500000', 'xyaxes': '0.00000000 -1.00000000 0.00000000 0.67267279 0.00000000 -0.73994007'}}
        ET.SubElement(body, "camera", name=f"{side}_wrist", **wrist_view[side], fovy="85")
        for finger in (1,2):
            b = world.find(f".//body[@name='{side}_gripper_finger_link{finger}']")
            for g in b.findall("geom"):
                if g.get("contype") != "0":
                    g.set("friction", "1.2 0.005 0.0001")
                    g.set("condim", "4")
    ET.SubElement(world, "camera", name="front", pos="1.8 -1.7 1.8",
                  xyaxes="0.85 0.526 0 -0.21 0.34 0.916", fovy="55")
    # Fixed task cameras: calibrated simulation views, not claimed real intrinsics.
    ET.SubElement(world, "camera", name="overview", pos="0.1 0 2.8",
                  xyaxes="0.000000000 -1.000000000 0.000000000 0.911921505 0.000000000 0.410364677", fovy="65")
    ET.SubElement(world, "camera", name="head", pos="0.05 0 1.6",
                  xyaxes="0 -1 0 0.73 0 0.684", fovy="80")
    # Destination marker has no collision and is not a success label by itself.
    ET.SubElement(world, "site", name="place_target", pos="0.85 0.32 0.802",
                  type="box", size="0.28 0.215 0.001", rgba="0.1 0.8 0.3 0.2", group="4")
    ET.indent(tree)
    output = scene_dir / "scene_physics.xml"
    tree.write(output, encoding="utf-8")
    model = mujoco.MjModel.from_xml_path(str(output))
    report = {"nq": model.nq, "nv": model.nv, "nu": model.nu, "ncam": model.ncam,
              "collision_parts": {"tray_runtime":len(manifest["runtime_tray_parts"]),
                                  "tray_detailed_asset":len(manifest["parts"]["tray"]),
                                  "table":len(manifest["parts"]["table"])},
              "tray_mass_kg": 1.2, "mass_and_friction": "simulation assumptions; unmeasured",
              "scene_sha256": hashlib.sha256(output.read_bytes()).hexdigest()}
    (scene_dir / "physics_build.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--scene-dir", type=Path, default=ROOT / "outputs/r1pro_tray_scene")
    p.add_argument("--rebuild-collisions", action="store_true")
    args = p.parse_args()
    build(args.scene_dir, args.rebuild_collisions)
