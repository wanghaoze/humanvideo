"""Create a portable, checksummed server/Quest package without virtual environments."""
from pathlib import Path
import hashlib
import json
import shutil
import zipfile

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/"deliverables/r1pro-tray-server.zip"
files={}
def add(path,target=None):
    path=Path(path)
    files[target or str(path.relative_to(ROOT)).replace("\\","/")]=path

for path in (ROOT/"r1pro_teleop").rglob("*"):
    if path.is_file() and "__pycache__" not in path.parts:add(path)
for name in ["setup_r1pro_server.sh","run_r1pro_server.sh","install_quest_apk.ps1","fetch_quest_apk.py",
             "control_r1pro_service.py","check_r1pro_http.py","check_r1pro_runtime.py",
             "select_r1pro_gpu.py","run_r1pro_gpu.sh"]:
    add(ROOT/"scripts"/name)
add(ROOT/"requirements-r1pro-sim.txt")
add(ROOT/"R1PRO_QUEST_SERVER.md","README.md")
add(ROOT/"R1PRO_SERVER_OPTIMIZATION.md")
add(ROOT/"tests/test_r1pro_pipeline.py")
add(ROOT/"tests/test_r1pro_quality.py")
for path in (ROOT/"deliverables/quest3").iterdir():
    if path.is_file():add(path)
scene=ROOT/"outputs/r1pro_tray_scene"
for name in ("scene.xml","scene_physics.xml","object_dimensions.json","physics_build.json"):
    add(scene/name)
add(scene/"scene_workspace_candidate.xml")
for path in (scene/"assets").rglob("*"):
    if path.is_file() and path.suffix.lower() not in (".usdc",".urdf"):
        # Omit superseded table collision iterations; retain detailed and runtime tray meshes.
        if path.name.startswith("table_collision_") and not path.name.startswith("table_collision_v2_"):continue
        add(path)
for path in (ROOT/"runs/r1pro_smoke_v4").glob("*.png"):
    add(path,"validation/"+path.name)
add(ROOT/"runs/r1pro_smoke_v4/smoke_report.json","validation/smoke_report.json")
add(ROOT/"runs/r1pro_http_acceptance.json","validation/http_acceptance.json")
add(ROOT/"runs/r1pro_lerobot_smoke_v2/meta/info.json","validation/lerobot_info.json")
add(ROOT/"runs/r1pro_export_validation.log","validation/lerobot_export.log")
if (ROOT/"runs/r1pro_unit_tests.txt").exists():
    add(ROOT/"runs/r1pro_unit_tests.txt","validation/unit_tests.txt")
for path in (ROOT/"runs/server_validation").iterdir():
    if path.is_file():add(path,"validation/server/"+path.name)
for path in (ROOT/"runs/server_optimization_20260914").iterdir():
    if path.is_file():add(path,"validation/optimization_20260914/"+path.name)
manifest={"package":"r1pro-tray-server","version":"0.2.0","date":"2026-09-14",
          "validated_on":"Linux Conda Python 3.11 / NVIDIA EGL GPU 0 / MuJoCo 3.13 / LeRobot 0.4.4 / torch 2.8 cu129; Windows HTTP client",
          "pending":["Quest 3 hardware tracking/installation","online IK and collision-checked paths for candidate layout","complete tray grasp"],
          "files":{key:hashlib.sha256(path.read_bytes()).hexdigest() for key,path in sorted(files.items())}}
OUT.parent.mkdir(exist_ok=True)
with zipfile.ZipFile(OUT,"w",compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    for key,path in sorted(files.items()):z.write(path,"r1pro-tray-server/"+key)
    z.writestr("r1pro-tray-server/package_manifest.json",json.dumps(manifest,indent=2))
report={"file":str(OUT),"bytes":OUT.stat().st_size,
        "sha256":hashlib.sha256(OUT.read_bytes()).hexdigest(),"files":len(files)}
OUT.with_suffix(".sha256").write_text(report["sha256"]+"  "+OUT.name+"\n")
print(json.dumps(report,indent=2))
