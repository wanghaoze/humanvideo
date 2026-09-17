"""Synchronized airport simulation evidence. Privileged signals never policy observations."""
import json,hashlib
from pathlib import Path
import numpy as np
import mujoco
from .core import Recorder,Simulation,XR_TO_ROBOT

class AirportSimulation(Simulation):
    def __init__(self,scene,*args,**kwargs):
        super().__init__(scene,*args,**kwargs)
        self.workspace_rotation=self.data.body('base_link').xmat.reshape(3,3).copy()
        self.xr_to_world=self.workspace_rotation@XR_TO_ROBOT
        self.wrist_guard_active=False
    @property
    def base_control_available(self):
        return mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_JOINT,'agent_base_x')>=0
    def base_joint_state(self):
        return np.array([self.data.qpos[self.model.jnt_qposadr[self.model.joint('agent_base_'+n).id]] for n in ('x','y','yaw')])
    def base_state(self):
        b=self.data.body('base_link');r=b.xmat.reshape(3,3)
        return np.array([b.xpos[0],b.xpos[1],np.arctan2(r[1,0],r[0,0])],np.float32)
    def set_base_world_target(self,goal):
        origin=self.model.body_pos[self.model.body('base_link').id]
        q=self.model.body_quat[self.model.body('base_link').id]
        initial_yaw=2*np.arctan2(q[3],q[0])
        angle=goal[2]-initial_yaw
        current=self.base_joint_state()[2]
        angle=current+np.arctan2(np.sin(angle-current),np.cos(angle-current))
        self.base_target=np.array([goal[0]-origin[0],goal[1]-origin[1],angle],float)
    def reset(self):
        self.base_target=np.zeros(3)
        super().reset()
    def apply_target(self):
        super().apply_target()
        if self.base_control_available:
            for i,n in enumerate(('x','y','yaw')):
                self.data.ctrl[self.model.actuator('agent_base_'+n+'_target').id]=self.base_target[i]
    def step(self):
        super().step()
        self.workspace_rotation=self.data.body('base_link').xmat.reshape(3,3).copy()
        self.xr_to_world=self.workspace_rotation@XR_TO_ROBOT
    def image(self,name="overview"):
        if name!='overview':return super().image(name)
        camera=mujoco.MjvCamera();camera.lookat[:]=[-.1,-.5,.6];camera.distance=5.;camera.azimuth=145;camera.elevation=-48
        self.renderer.update_scene(self.data,camera=camera,scene_option=self.options)
        return self.renderer.render().copy()
    def update_control(self,now=None):
        previous=self.target.copy();super().update_control(now)
        self.ik_data.qpos[:]=self.data.qpos
        for side,offset in [('left',0),('right',8)]:
            ids=[self.model.jnt_qposadr[j] for j in self.arms[side]]
            self.ik_data.qpos[ids]=self.target[offset:offset+7]
        mujoco.mj_forward(self.model,self.ik_data)
        safe=all(float(self.ik_data.site(side+'_tcp').xmat.reshape(3,3)[:,1]@self.workspace_rotation[:,1])>0 for side in ('left','right'))
        self.wrist_guard_active=not safe
        if not safe:self.target=previous;self.anchors.clear()

class Evidence:
    def __init__(self,sim,tray_names=None):
        self.sim=sim;self.names=tray_names or ['tray_0','tray_1','tray_2']
        self.ids={sim.model.body(n).id:i for i,n in enumerate(self.names)}
        self.table_ids={b for b in range(sim.model.nbody) if sim.model.body(b).name in ('front_left','front_right','rear','table')}
        self.local_vertices={}
        for bid in self.ids:
            verts=[]
            body=sim.data.body(bid)
            for g in range(sim.model.ngeom):
                if sim.model.geom_bodyid[g]!=bid or not sim.model.geom_contype[g]:continue
                mesh=int(sim.model.geom_dataid[g])
                if mesh<0:continue
                start=int(sim.model.mesh_vertadr[mesh]);num=int(sim.model.mesh_vertnum[mesh])
                world=sim.model.mesh_vert[start:start+num]@sim.data.geom_xmat[g].reshape(3,3).T+sim.data.geom_xpos[g]
                verts.extend((world-body.xpos)@body.xmat.reshape(3,3))
            self.local_vertices[bid]=np.asarray(verts)
    def row(self):
        s=self.sim;n=len(self.names);poses=np.zeros((n,7));forces=np.zeros((n,2));table=np.zeros(n);support=np.zeros(n);floor=np.zeros(n);bottom=np.zeros(n);bad=0;penetration=0.
        for bid,i in self.ids.items():
            b=s.data.body(bid);poses[i]=np.r_[b.xpos,b.xquat]
            bottom[i]=(self.local_vertices[bid]@b.xmat.reshape(3,3).T+b.xpos)[:,2].min()
        for k,c in enumerate(s.data.contact):
            a,b=[int(s.model.geom_bodyid[g]) for g in (c.geom1,c.geom2)];names=[s.model.body(x).name for x in (a,b)]
            force=np.zeros(6);mujoco.mj_contactForce(s.model,s.data,k,force)
            if force[0]<=.05:continue
            if a in self.ids or b in self.ids:
                penetration=max(penetration,float(-c.dist))
                for bid,other in ((a,b),(b,a)):
                    if bid not in self.ids:continue
                    i=self.ids[bid];other_name=s.model.body(other).name
                    for side,j in [('left',0),('right',1)]:
                        if other_name.startswith(side+'_gripper_finger'):forces[i,j]+=force[0]
                    table[i]+=int(other in self.table_ids)
                    support[i]+=int(other in self.table_ids or (other in self.ids and poses[self.ids[other],2]<poses[i,2]))
                    floor[i]+=int(other==0)
                    if other_name.startswith(('left_','right_','torso','base_')) and 'gripper_finger' not in other_name:bad+=1
            elif any(x.startswith(('left_','right_','torso')) for x in names):bad+=1
        tcp=np.array([s.tcp(side)[0] for side in ('left','right')])
        return {'privileged/tray_pose':poses,'privileged/gripper_contact_force':forces,'privileged/table_contact':table,'privileged/support_contact':support,'privileged/floor_contact':floor,'privileged/min_z':bottom,'privileged/illegal_collision':np.int32(bad),'privileged/max_penetration':np.float64(penetration),'privileged/tcp':tcp}

class AirportRecorder(Recorder):
    def __init__(self,directory,sim,spec,controller_source):
        if controller_source not in ('human_teleoperation','program_generated'):raise ValueError('Explicit controller_source required')
        super().__init__(directory,sim,task='Arrange all three airport trays in left-aligned belt slots',controller_source=controller_source,task_kind='airport_arrangement',tray_body='tray_0',goal_site=None)
        self.evidence=Evidence(sim)
        self.file.attrs.update(scene_id=spec['scene_id'],scene_json=json.dumps(spec),base_control_available=sim.base_control_available,base_model="simulation_ideal_planar_position_servo_v1" if sim.base_control_available else "fixed",environment_source='simulation',privileged_keys=json.dumps(['qpos','qvel','qacc_warmstart','tray_pose','base_state','base_joint_state','privileged/*']),initial_sim_time=float(sim.data.time),operator_result='unlabeled',annotation_version='airport_events/1.0')
    def append(self,sim,images,input_age,extras=None):
        data=self.evidence.row();data.update(extras or {})
        if sim.base_control_available:
            data.update(base_state=sim.base_state(),base_action=sim.base_target.astype(np.float32),base_joint_state=sim.base_joint_state().astype(np.float32))
            self.file.attrs['base_action_semantics']='absolute planar joint targets: world X/Y displacement m from initial origin, yaw offset rad from initial heading; simulated servo only'
        super().append(sim,images,input_age,data)
    def finish(self,result='unlabeled'):
        self.file.attrs['operator_result']=result
        self.file.attrs['abort_requested']=result=='aborted'
        # Outcome is assigned only by the centralized physical criteria afterward.
        return super().finish('unlabeled')
