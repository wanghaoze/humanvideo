"""Simulation-only joint targets, relative controller IK, and synchronized recording."""
from pathlib import Path
import hashlib
import json
import time
import uuid

import h5py
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

CAMERAS = ("head", "left_wrist", "right_wrist")
NAMES = ([f"left_arm_joint{i}" for i in range(1,8)] + ["left_gripper_opening_m"] +
         [f"right_arm_joint{i}" for i in range(1,8)] + ["right_gripper_opening_m"])
XR_TO_ROBOT = np.array([[0.,0.,-1.],[-1.,0.,0.],[0.,1.,0.]])


def validate_packet(packet):
    if not isinstance(packet, dict) or not isinstance(packet.get("seq"), int):
        raise ValueError("packet requires an integer seq")
    hands = packet.get("hands", {})
    if not isinstance(hands, dict):
        raise ValueError("hands must be an object")
    clean = {"seq": packet["seq"], "hands": {}, "buttons": {}}
    for side in ("left", "right"):
        if side not in hands:
            continue
        h = hands[side]
        p = np.asarray(h["position"], dtype=float)
        q = np.asarray(h["quaternion_xyzw"], dtype=float)
        if p.shape != (3,) or q.shape != (4,) or not np.all(np.isfinite(np.r_[p,q])):
            raise ValueError("invalid controller pose")
        if np.linalg.norm(p)>20 or abs(np.linalg.norm(q)-1)>0.05:
            raise ValueError("pose outside expected OpenXR tracking coordinates")
        grip, trigger = float(h["grip"]), float(h["trigger"])
        if not 0<=grip<=1 or not 0<=trigger<=1:
            raise ValueError("grip/trigger outside [0,1]")
        clean["hands"][side] = {"position": p, "rotation": Rotation.from_quat(q).as_matrix(),
                                 "grip": grip, "trigger": trigger}
    for key in ("A","B","X","Y"):
        clean["buttons"][key] = packet.get("buttons", {}).get(key) is True
    return clean


class Simulation:
    def __init__(self, scene, fps=20, render=True, width=320, height=240):
        self.scene = Path(scene).resolve()
        self.model = mujoco.MjModel.from_xml_path(str(self.scene))
        self.data = mujoco.MjData(self.model)
        self.ik_data = mujoco.MjData(self.model)
        self.fps = fps
        self.nstep = round(1 / fps / self.model.opt.timestep)
        if abs(self.nstep * self.model.opt.timestep * fps - 1) > 1e-8:
            raise ValueError("fps must divide physics frequency exactly (default 20 Hz)")
        self.arms, self.fingers = {}, {}
        for side in ("left", "right"):
            self.arms[side] = [self.model.joint(f"{side}_arm_joint{i}").id for i in range(1,8)]
            self.fingers[side] = [self.model.joint(f"{side}_gripper_finger_joint{i}").id for i in (1,2)]
        self.anchors = {}
        self.filtered = {}
        self.joint_velocity = {}
        self.position_gain = 1.5
        self.rotation_gain = 1.3
        self.last_packet_seq = -1
        self.last_packet_time = 0.
        self.latest = None
        self.paused = False
        self.width, self.height = width, height
        self.renderer = mujoco.Renderer(self.model, height=height, width=width) if render else None
        self.options = mujoco.MjvOption()
        self.options.geomgroup[3:] = 0
        self.reset()

    def reset(self):
        home=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_KEY,"teleop_home")
        if home>=0:
            mujoco.mj_resetDataKeyframe(self.model,self.data,home)
        else:
            mujoco.mj_resetData(self.model, self.data)
            for side in ("left", "right"):
                elbow = self.model.joint(f"{side}_arm_joint4").id
                self.data.qpos[self.model.jnt_qposadr[elbow]] = -np.pi / 2
                for j in self.fingers[side]:
                    self.data.qpos[self.model.jnt_qposadr[j]] = 0.05
        mujoco.mj_forward(self.model, self.data)
        self.target = self.state()
        self.anchors.clear()
        self.filtered.clear()
        self.joint_velocity.clear()
        self.latest = None
        self.last_packet_seq = -1
        self.last_packet_time = 0.
        self.paused = False
        self.apply_target()

    def state(self):
        result = []
        for side in ("left","right"):
            result.extend(self.data.qpos[self.model.jnt_qposadr[j]] for j in self.arms[side])
            result.append(sum(self.data.qpos[self.model.jnt_qposadr[j]] for j in self.fingers[side]))
        return np.asarray(result, dtype=np.float32)

    def input(self, packet, now=None):
        packet = validate_packet(packet)
        if packet["seq"] <= self.last_packet_seq:
            return False
        self.latest = packet
        self.last_packet_seq = packet["seq"]
        self.last_packet_time = time.monotonic() if now is None else now
        return True

    def disconnect(self):
        self.latest = None
        self.anchors.clear()
        self.filtered.clear()
        self.joint_velocity.clear()
        self.last_packet_seq = -1
        self.target = self.state()

    def tcp(self, side):
        s = self.data.site(f"{side}_tcp")
        return s.xpos.copy(), s.xmat.reshape(3,3).copy()

    def ik(self, side, pos, rot):
        ids = self.arms[side]
        qids = np.array([self.model.jnt_qposadr[j] for j in ids])
        dofs = np.array([self.model.jnt_dofadr[j] for j in ids])
        d = self.ik_data
        d.qpos[:] = self.data.qpos
        start = 0 if side=="left" else 8
        d.qpos[qids] = self.target[start:start+7]
        site_id = self.model.site(f"{side}_tcp").id
        jp, jr = np.zeros((3,self.model.nv)), np.zeros((3,self.model.nv))
        for _ in range(24):
            mujoco.mj_forward(self.model, d)
            actual_rot = d.site_xmat[site_id].reshape(3,3)
            err = np.r_[pos-d.site_xpos[site_id], 0.5*Rotation.from_matrix(rot @ actual_rot.T).as_rotvec()]
            if np.linalg.norm(err) < 0.001:
                break
            mujoco.mj_jacSite(self.model, d, jp, jr, site_id)
            jac = np.vstack([jp[:,dofs],0.5*jr[:,dofs]])
            dq = jac.T @ np.linalg.solve(jac @ jac.T + np.eye(6)*0.001, err)
            d.qpos[qids] += np.clip(dq,-0.08,0.08)
            d.qpos[qids] = np.clip(d.qpos[qids], self.model.jnt_range[ids,0]+0.01,
                                   self.model.jnt_range[ids,1]-0.01)
        # Bound target change even if IK cannot reach an operator's request.
        old = self.target[start:start+7]
        limits = np.array([1.5,1.5,1.5,1.5,2.2,2.2,2.2])
        desired = np.clip((d.qpos[qids]-old)*self.fps,-limits,limits)
        previous = self.joint_velocity.get(side,np.zeros(7))
        velocity = previous + np.clip(desired-previous,-8.0/self.fps,8.0/self.fps)
        self.joint_velocity[side] = velocity
        return np.clip(old+velocity/self.fps,self.model.jnt_range[ids,0]+0.01,
                       self.model.jnt_range[ids,1]-0.01)

    def update_control(self, now=None):
        now = time.monotonic() if now is None else now
        if self.paused or self.latest is None or now-self.last_packet_time > 0.25:
            self.anchors.clear();self.filtered.clear();self.joint_velocity.clear()
            return
        for side in ("left", "right"):
            h = self.latest["hands"].get(side)
            # Hysteresis avoids repeated anchor resets near the grip threshold.
            threshold = 0.35 if side in self.anchors else 0.6
            if h is None or h["grip"] < threshold:
                self.anchors.pop(side,None);self.filtered.pop(side,None)
                self.joint_velocity.pop(side,None)
                continue
            if side not in self.anchors:
                p,r = self.tcp(side)
                self.anchors[side] = (h["position"].copy(),h["rotation"].copy(),p,r)
                self.filtered[side] = (h["position"].copy(),h["rotation"].copy(),now)
                self.joint_velocity.pop(side,None)
            fp,fr,stamp = self.filtered[side]
            dt = np.clip(now-stamp,0.001,0.1)
            dp = h["position"]-fp
            dr = Rotation.from_matrix(h["rotation"] @ fr.T).as_rotvec()
            # Small tracking noise is held; intentional motion gets less smoothing.
            distance,angle = np.linalg.norm(dp),np.linalg.norm(dr)
            alpha = 1-np.exp(-dt/(0.025 if distance>0.015 else 0.075))
            if distance>0.0015:fp=fp+alpha*dp
            alpha_r = 1-np.exp(-dt/(0.025 if angle>0.1 else 0.065))
            if angle>np.deg2rad(0.5):fr=Rotation.from_rotvec(alpha_r*dr).as_matrix() @ fr
            self.filtered[side]=(fp,fr,now)
            hp,hr,rp,rr = self.anchors[side]
            goal = rp + self.position_gain*(XR_TO_ROBOT @ (fp-hp))
            # Translate the existing simulation workspace with the robot base.
            base = self.data.body("base_link").xpos
            goal = np.clip(goal,base+[-0.35,-0.9,0.35],base+[1.1,0.9,1.8])
            relative = Rotation.from_matrix(fr @ hr.T).as_rotvec()*self.rotation_gain
            rotation = XR_TO_ROBOT @ Rotation.from_rotvec(relative).as_matrix() @ XR_TO_ROBOT.T @ rr
            start = 0 if side=="left" else 8
            self.target[start:start+7] = self.ik(side,goal,rotation)
            opening = 0.1 - 0.096*h["trigger"]
            self.target[start+7] += np.clip(opening-self.target[start+7],-0.006,0.006)

    def apply_target(self):
        for side,start in (("left",0),("right",8)):
            for i in range(7):
                self.data.ctrl[self.model.actuator(f"{side}_arm_joint{i+1}_target").id] = self.target[start+i]
            self.data.ctrl[self.model.actuator(f"{side}_gripper_target").id] = self.target[start+7]/2

    def step(self):
        self.apply_target()
        for _ in range(self.nstep):
            mujoco.mj_step(self.model,self.data)
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            raise RuntimeError("nonfinite simulation state")

    def images(self):
        return {name:self.image(name) for name in CAMERAS}

    def image(self, name="overview"):
        if self.renderer is None:
            raise RuntimeError("renderer disabled")
        self.renderer.update_scene(self.data,camera=name,scene_option=self.options)
        self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
        self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False
        return self.renderer.render().copy()

    def close(self):
        if self.renderer:
            self.renderer.close()


class Recorder:
    def __init__(self, directory, sim, task="Move the empty tray to the marked area", session_id=None, controller_source="unspecified", task_kind="tray_transfer"):
        directory = Path(directory)
        directory.mkdir(parents=True,exist_ok=True)
        self.stem = time.strftime("episode_%Y%m%d_%H%M%S_")+uuid.uuid4().hex[:8]
        self.path = directory / (self.stem+".partial.h5")
        self.file = h5py.File(self.path,"x")
        self.file.attrs.update(schema="r1pro_sim_raw_v1",source="simulation",fps=sim.fps,
            task=task,session_id=session_id or uuid.uuid4().hex,
            scene_sha256=hashlib.sha256(sim.scene.read_bytes()).hexdigest(),
            names=json.dumps(NAMES),camera_names=json.dumps(CAMERAS),
            action_semantics="absolute joint targets rad; gripper summed slide opening m",
            observation_semantics="state and RGB at t BEFORE applying action[t] over next interval",
            result="incomplete",success_operator=False,controller_source=controller_source,task_kind=task_kind)
        goal=sim.model.site('place_target').pos.copy()
        goal[2]-=.002  # Marker is 2 mm above the tabletop in the supplied scenes.
        self.file.attrs['task_goal_xyz']=json.dumps(goal.tolist())
        self.count=0
        self.t0=float(sim.data.time)

    def append(self,sim,images,input_age,extras=None):
        row = {"state":sim.state(),"action":sim.target.astype(np.float32).copy(),
               "qpos":sim.data.qpos.copy(),"qvel":sim.data.qvel.copy(),
               "qacc_warmstart":sim.data.qacc_warmstart.copy(),
               "timestamp":np.float64(sim.data.time-self.t0),
               "wall_monotonic":np.float64(time.monotonic()),
               "input_age_s":np.float32(input_age),
               "tray_pose":np.r_[sim.data.body("tray").xpos,sim.data.body("tray").xquat]}
        row.update({f"images/{k}":v for k,v in images.items()})
        if extras:row.update(extras)
        for k,v in row.items():
            v=np.asarray(v)
            if k not in self.file:
                self.file.create_dataset(k,shape=(0,)+v.shape,maxshape=(None,)+v.shape,
                                         chunks=(1,)+v.shape,dtype=v.dtype,
                                         compression="lzf" if v.ndim else None)
            ds=self.file[k];ds.resize(self.count+1,axis=0);ds[self.count]=v
        self.count+=1
        if self.count%20==0:self.file.flush()

    def finish(self,result="unlabeled"):
        if result not in ("success","failure","unlabeled","smoke"):
            raise ValueError("invalid result")
        self.file.attrs["result"]=result
        self.file.attrs["success_operator"]=(result=="success" and self.file.attrs.get("controller_source")!="program_generated")
        self.file.attrs["num_frames"]=self.count
        self.file.flush();self.file.close()
        final=self.path.with_name(self.stem+".h5")
        self.path.rename(final)
        return final
