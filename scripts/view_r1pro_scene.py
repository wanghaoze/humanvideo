"""Interactive static MuJoCo viewer, PNG rendering, and USD visual export."""
import argparse
from pathlib import Path
import time
import mujoco
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/r1pro_tray_scene'
parser=argparse.ArgumentParser(__doc__)
parser.add_argument('--render', action='store_true')
parser.add_argument('--export-usd', action='store_true')
args=parser.parse_args()
m=mujoco.MjModel.from_xml_path(str(OUT/'scene.xml'))
d=mujoco.MjData(m)
mujoco.mj_forward(m,d)
opt=mujoco.MjvOption()
opt.geomgroup[3:]=0
cam=mujoco.MjvCamera()
cam.lookat[:]=[0.45,0,0.85]
cam.distance=3.4
cam.azimuth=135
cam.elevation=-22
if args.render:
    from PIL import Image
    with mujoco.Renderer(m,height=1000,width=1600) as renderer:
        renderer.update_scene(d,camera=cam,scene_option=opt)
        Image.fromarray(renderer.render()).save(OUT/'preview.png')
if args.export_usd:
    from mujoco.usd.exporter import USDExporter
    from pxr import Usd, UsdGeom
    exporter=USDExporter(m,output_directory='usd',output_directory_root=str(OUT))
    exporter.update_scene(d,scene_option=opt)
    UsdGeom.SetStageMetersPerUnit(exporter.stage,1.0)
    exporter.stage.SetEndTimeCode(0)
    exporter.stage.GetRootLayer().Export(str(OUT/'usd/scene.usdc'))
    stage=Usd.Stage.Open(str(OUT/'usd/scene.usdc'))
    assert stage and stage.GetDefaultPrim()
    print('USD validated:', len(list(stage.Traverse())), 'prims')
if not args.render and not args.export_usd:
    import mujoco.viewer
    with mujoco.viewer.launch_passive(m,d) as viewer:
        viewer.cam.lookat[:]=cam.lookat
        viewer.cam.distance=cam.distance
        viewer.cam.azimuth=cam.azimuth
        viewer.cam.elevation=cam.elevation
        viewer.opt.geomgroup[:]=opt.geomgroup
        while viewer.is_running():
            mujoco.mj_forward(m,d)
            viewer.sync()
            time.sleep(1/60)
