"""Closed-loop program-generated tray arrangement in simulation only."""
import argparse,json,time
from pathlib import Path
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation,Slerp
from .airport_recording import AirportSimulation,Evidence
from .airport_mobile import move_base,collision_pairs
from scipy.optimize import least_squares

def solve(s,side,pos,rot,seed):
    d=s.ik_data;d.qpos[:]=s.data.qpos;ids=s.arms[side]
    qids=[s.model.jnt_qposadr[j] for j in ids];bounds=s.model.jnt_range[ids]
    def fun(q):
        d.qpos[qids]=q;mujoco.mj_kinematics(s.model,d);site=d.site(side+'_tcp')
        return np.r_[site.xpos-pos,.4*Rotation.from_matrix(rot@site.xmat.reshape(3,3).T).as_rotvec(),.003*(q-seed)]
    fit=least_squares(fun,np.clip(seed,bounds[:,0]+.01,bounds[:,1]-.01),bounds=(bounds[:,0]+.01,bounds[:,1]-.01),max_nfev=100)
    return fit.x,np.linalg.norm(fun(fit.x)[:6])



class Controller:
    def __init__(self,scene,output):
        self.sim=AirportSimulation(scene,render=False)
        self.output=Path(output);self.output.mkdir(parents=True,exist_ok=False)
        self.evidence=Evidence(self.sim);self.rows=[];self.events=[];self.target_tray=-1
        self.home=self.sim.target.copy();self.scene=Path(scene)

    def capture(self,stage):
        s=self.sim;s.target=s.target.astype(np.float32);s.base_target=s.base_target.astype(np.float32);e=self.evidence.row()
        row=dict(qpos=s.data.qpos.copy(),qvel=s.data.qvel.copy(),warm=s.data.qacc_warmstart.copy(),
                 action=s.target.copy(),base_action=s.base_target.copy(),state=s.state(),base_state=s.base_state(),
                 time=float(s.data.time),stage=stage,target_tray=self.target_tray)
        row.update({k.split('/')[-1]:v for k,v in e.items()});self.rows.append(row)

    def tick(self,stage):
        self.capture(stage);self.sim.step()
        bad=collision_pairs(self.sim)
        if bad:raise RuntimeError(stage+': collision '+json.dumps(bad[:3]))
        e=self.evidence.row()
        if np.any(e['privileged/floor_contact']):raise RuntimeError(stage+': tray on floor')
        if e['privileged/max_penetration']>.003:raise RuntimeError(stage+': penetration '+str(e['privileged/max_penetration']))

    def log(self,stage):
        row=dict(frame=len(self.rows),stage=stage,base=self.sim.base_state().tolist(),trays=[self.sim.data.body('tray_'+str(i)).xpos.tolist() for i in range(3)])
        self.events.append(row);print(json.dumps(row),flush=True)

    def joints(self,target,seconds,stage):
        old=self.sim.target.copy();steps=max(round(seconds*20),int(np.max(abs(target-old))/.025)+1)
        for i in range(steps):
            u=(i+1)/steps;u=u*u*(3-2*u);self.sim.target=old+(target-old)*u;self.tick(stage)
        self.log(stage)

    def base(self,x,y,yaw):
        move_base(self.sim,np.array([x,y,yaw]),self.capture,speed=.12,yaw_speed=.25);self.log('align_base')

    def cartesian(self,center,angle,seconds,stage,opening=None,sides=None):
        s=self.sim;rotation=Rotation.from_euler('z',angle).as_matrix();ends={};rotations={};starts={}
        for side,sign in [('left',1),('right',-1)]:
            if sides and side not in sides:continue
            ends[side]=np.array(center)+rotation@np.array([0,.205*sign,0])
            endrot=rotation@(Rotation.from_euler('y',-30,degrees=True)*Rotation.from_euler('x',-10*sign,degrees=True)).as_matrix()
            starts[side],r=s.tcp(side);rotations[side]=Slerp([0,1],Rotation.from_matrix(np.stack([r,endrot])))
        steps=round(seconds*20)
        for i in range(0,steps,4):
            old=s.target.copy();end=old.copy();u=min(1,(i+4)/steps);u=u*u*(3-2*u)
            for side,offset in [('left',0),('right',8)]:
                if sides and side not in sides:continue
                pos=starts[side]+u*(ends[side]-starts[side]);q,error=solve(s,side,pos,rotations[side](u).as_matrix(),old[offset:offset+7])
                if error>.025:raise RuntimeError(stage+': IK '+side+' error '+str(error))
                if np.max(abs(q-old[offset:offset+7]))>.5:raise RuntimeError(stage+': IK branch jump '+str(q-old[offset:offset+7]))
                end[offset:offset+7]=q
            if opening is not None:
                for side,j in [('left',7),('right',15)]:
                    if not sides or side in sides:end[j]=opening
            subdivisions=max(4,int(np.ceil(np.max(abs(end-old))/.07)))
            for j in range(subdivisions):
                s.target=old+(end-old)*((j+1)/subdivisions);self.tick(stage)
        self.log(stage)

    def grasp(self,tray,base_yaw=0.):
        s=self.sim;self.target_tray=tray
        body=s.data.body('tray_'+str(tray));p=body.xpos.copy();r=body.xmat.reshape(3,3);yaw=np.arctan2(r[1,0],r[0,0])
        # Rectangular tray rims can be grasped from either end without wrist inversion.
        angle=base_yaw+((yaw-base_yaw+np.pi/2)%np.pi-np.pi/2)
        approach=p.copy();approach[2]=.98;approach[:2]+=.00*np.array([np.cos(base_yaw),np.sin(base_yaw)])
        self.cartesian(approach,angle,8,'pregrasp')
        contact=p.copy();contact[2]+=.069
        self.cartesian(contact,angle,3,'approach')
        close=s.target.copy();close[[7,15]]=.004;self.joints(close,2,'close')
        e=self.evidence.row()
        if np.any(e['privileged/gripper_contact_force'][tray]<.5):raise RuntimeError('close: missing bilateral contact '+str(e['privileged/gripper_contact_force'][tray]))
        lift=contact.copy();lift[2]=1.02;lift[:2]+=.00*np.array([np.cos(base_yaw),np.sin(base_yaw)])
        self.cartesian(lift,angle,4,'lift',.004)
        self.joints(s.target.copy(),1,'stabilize')
        e=self.evidence.row()
        if e['privileged/table_contact'][tray] or e['privileged/min_z'][tray]<.84:raise RuntimeError('lift: tray not airborne')
        return angle

    def separate_rear(self,tray,face):
        s=self.sim;p=s.data.body('tray_'+str(tray)).xpos.copy()
        center=p.copy();center[2]=.99
        self.cartesian(center,face,6,'adjust_approach',.1,['right'])
        center[2]=.869;self.cartesian(center,face,3,'adjust_contact',.1,['right'])
        close=s.target.copy();close[15]=.004;self.joints(close,2,'adjust_close')
        center[1]+=.14;self.cartesian(center,face,5,'push_adjust_tray',.004,['right'])
        opened=s.target.copy();opened[15]=.1;self.joints(opened,2,'adjust_release')
        center[2]=1.0;self.cartesian(center,face,3,'adjust_retreat',.1,['right'])
        self.joints(self.tucked,3,'adjust_home')

    def nudge_slot(self,tray,goal_y):
        s=self.sim;p=s.data.body('tray_'+str(tray)).xpos.copy();center=p.copy();center[2]=.99
        self.cartesian(center,0.,5,'slot_adjust_approach',.1,['right'])
        center[2]=.885;self.cartesian(center,0.,3,'slot_adjust_contact',.1,['right'])
        closed=s.target.copy();closed[15]=.004;self.joints(closed,2,'slot_adjust_close')
        center[1]=goal_y;self.cartesian(center,0.,5,'push_adjust_tray',.004,['right'])
        for _ in range(3):
            actual=s.data.body('tray_'+str(tray)).xpos.copy();error=np.array([.85,goal_y])-actual[:2]
            if abs(error[1])<.006:break
            center[1]+=np.clip(error[1],-.025,.025);self.cartesian(center,0.,2,'push_adjust_correct',.004,['right'])
        opened=s.target.copy();opened[15]=.1;self.joints(opened,2,'slot_adjust_release')
        center[2]=1.;self.cartesian(center,0.,3,'slot_adjust_retreat',.1,['right'])
        self.joints(self.tucked,3,'return_home')

    def run(self,only_first=False):
        s=self.sim
        self.joints(self.home,1,'settle')
        tucked=self.home.copy();tucked[[3,11]]=-1.70;tucked[[0,8]]=-.35;tucked[[5,13]]=.9
        self.joints(tucked,3,'navigation_clearance');self.tucked=tucked.copy()
        # The current bounded fixture is smoke 0009: one front tray, two rear trays.
        for slot,tray in enumerate((0,2,1)):
            self.target_tray=tray;p=s.data.body('tray_'+str(tray)).xpos.copy();rear=p[0]<0
            body=s.data.body('tray_'+str(tray));rr=body.xmat.reshape(3,3);yaw=np.arctan2(rr[1,0],rr[0,0]);face=(yaw+np.pi/2)%np.pi-np.pi/2+(np.pi if rear else 0.)
            pickup=p[:2]-.48*np.array([np.cos(face),np.sin(face)])
            if rear:
                self.base(-.3,s.base_state()[1],0.)
                self.base(-.3,float(p[1]),np.pi)
                self.base(float(pickup[0]),float(pickup[1]),face)
            else:self.base(float(pickup[0]),float(pickup[1]),face)
            if rear and tray==2:
                self.separate_rear(tray,face)
                p=s.data.body('tray_'+str(tray)).xpos.copy();pickup=p[:2]-.48*np.array([np.cos(face),np.sin(face)])
                self.base(float(pickup[0]),float(pickup[1]),face)
            angle=self.grasp(tray,face)
            if only_first:return
            # Lifted tray is clear of tabletops before turning and translating.
            if rear:
                self.base(-.3,float(p[1]),np.pi)
                self.base(-.3,float(p[1]),0.)
                angle-=np.pi
            goal_y=.6-(.005+.215+slot*(.43+.015))
            place_y=goal_y-(.10 if slot else 0.)
            current=s.base_state();self.base(-.1,float(current[1]),float(current[2]))
            self.base(-.1,place_y,0.)
            self.base(.28,place_y,0.)
            self.cartesian([.85,place_y,1.02],0.,5,'preplace',.004)
            self.cartesian([.85,place_y,.87],0.,4,'descend',.004)
            self.joints(s.target.copy(),1,'support_contact')
            if not self.evidence.row()['privileged/table_contact'][tray]:raise RuntimeError('place: no tabletop support')
            opened=s.target.copy();opened[[7,15]]=.1;self.joints(opened,2,'open')
            self.cartesian([.9,place_y,1.0],0.,3,'retreat',.1)
            self.joints(self.tucked,4,'return_home')
            if slot:self.nudge_slot(tray,goal_y)
        self.joints(self.tucked,2,'verify_placement')

    def save(self,error=None):
        if self.rows:
            np.savez_compressed(self.output/'trajectory.npz',**{k:np.array([r[k] for r in self.rows]) for k in self.rows[0]})
        report=dict(controller_source='program_generated',environment_source='simulation',scene=str(self.scene.resolve()),frames=len(self.rows),error=error,execution_completed=error is None,events=self.events)
        (self.output/'report.json').write_text(json.dumps(report,indent=2))
        self.sim.close();return report


def main():
    p=argparse.ArgumentParser();p.add_argument('--scene',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--only-first',action='store_true');a=p.parse_args()
    c=Controller(a.scene,a.output);error=None
    try:c.run(a.only_first)
    except Exception as e:error=str(e);print('FAILED:',error,flush=True)
    finally:c.save(error)

if __name__=='__main__':main()
