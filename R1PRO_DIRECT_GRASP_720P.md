# R1 Pro direct tray grasp, native 720p

This profile preserves the initial arms/torso from `vr_teleoperation_whole_body_launch.py`, tray mass, friction, and gripper gains. It replaces the old 180-degree wrist yaw with a direct grasp (yaw 0, pitch -30 degrees, symmetric 10-degree tilt). It moves the tray 15 cm forward while lifting to keep the arms within their reachable workspace.

The physical `left_realsense_link` and `right_realsense_link` must be ahead of their TCPs along world +X during grasp. Wrist local +Y must retain positive world +Y throughout the trajectory. The supplied initial pose is preserved; the camera-front criterion applies after approach, not to the initial resting pose. The cameras look outward and downward toward the tray.

Each LeRobot camera (`head`, `left_wrist`, `right_wrist`) is rendered at **1280 x 720**, 20 fps, H.264 CRF 18. This is native rendering, not enlargement of the old video. The four-view review is a separate 1280 x 720 mosaic at 5 fps. Live Quest monitoring retains its existing 640 x 480 preview for latency; new raw recording uses 720p.

## Server reproduction

Run from `/home/michelle/haoze/humanvideo/r1pro-tray-server` using the existing Conda environment:

```bash
conda activate r1pro-tray
export MUJOCO_GL=egl
export MUJOCO_EGL_DEVICE_ID=1
python scripts/run_tray_lift_pipeline.py --output runs/direct_grasp_720p_new
```

Choose a new output directory each time. The pipeline plans, validates an 8-second hold, re-executes synchronized actions/RGB, checks contacts and wrist orientation, exports via the official LeRobot 0.4.4 API, and independently replays the resulting episode.

Scene: `outputs/r1pro_tray_scene/scene_direct_grasp_720p.xml`. Rebuild it with `python scripts/build_direct_grasp_scene.py` after creating the supplied launch-pose scene.

Outputs below the run directory:

- `validated/report.json`: physics and physical camera/wrist checks.
- `dataset/lerobot`: LeRobot v3 metadata, Parquet state/actions, three 720p videos, source manifest.
- `dataset/raw`: synchronized simulation state, actions, RGB, contacts, physical camera checks in HDF5.
- `dataset/quality.json`: training-candidate checks on the actual recorded episode.
- `dataset/review.mp4` and `review_validation.json`: visual review and deterministic physics replay checks.

These episodes are labelled **program_generated simulation**, not human teleoperation or real-robot demonstrations. One successful synthetic episode validates this pipeline; it does not establish real-world grasp reliability or provide a complete training set.
