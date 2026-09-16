"""Record actual Linux GPU/runtime identities and test CUDA plus EGL rendering."""
import json
import os
from pathlib import Path
import sys
from importlib.metadata import version
import torch
from r1pro_teleop.core import Simulation
from OpenGL import GL

assert os.environ.get("CONDA_PREFIX") == sys.prefix
assert torch.cuda.is_available()
x = torch.randn(256, 256, device="cuda:0")
assert torch.isfinite(x @ x.T).all()
sim = Simulation("outputs/r1pro_tray_scene/scene_physics.xml")
try:
    frame = sim.image()
    report = {"python": sys.version, "conda_prefix": sys.prefix,
              "versions": {n: version(n) for n in ("mujoco", "torch", "torchvision", "lerobot", "torchcodec")},
              "torch_cuda": torch.version.cuda, "gpu": torch.cuda.get_device_name(0),
              "capability": torch.cuda.get_device_capability(0), "cuda_matmul": True,
              "gl_renderer": GL.glGetString(GL.GL_RENDERER).decode(),
              "gl_vendor": GL.glGetString(GL.GL_VENDOR).decode(),
              "render_nonempty": bool(frame.std() > 2)}
    assert "NVIDIA" in report["gl_vendor"]
    Path("runs/runtime_report.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
finally:
    sim.close()
