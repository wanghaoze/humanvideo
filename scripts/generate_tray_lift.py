"""Program-generated dual-arm tray lift in MuJoCo; never hardware or human demo."""
import argparse,json,sys,time,hashlib
from pathlib import Path
import numpy as np
import mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation,Slerp
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from r1pro_teleop.core import Simulation


def solve(sim,side,pos,rot,seed):
    d=sim.ik_data;d.qpos[:]=sim.data.qpos
    ids=sim.arms[side];qids=[sim.model.jnt_qposadr[j] for j in ids]
    limits=sim.model.jnt_range[ids];lo,hi=limits[:,0]+.01,limits[:,1]-.01
    def fun(q):
        d.qpos[qids]=q;mujoco.mj_kinematics(sim.model,d)
        site=d.site(side+'_tcp')
        return np.r_[site.xpos-pos,.4*Rotation.from_matrix(rot@site.xmat.reshape(3,3).T).as_rotvec()]
    best=None
    rng=np.random.default_rng(17)
    for initial in [seed]+[np.clip(seed+rng.normal(0,.7,7),lo,hi) for _ in range(7)]:
        fit=least_squares(fun,np.clip(initial,lo,hi),bounds=(lo,hi),max_nfev=160)
        error=np.linalg.norm(fun(fit.x));score=error+1e-5*np.linalg.norm(fit.x-seed)
        if best is None or score<best[0]:best=(score,fit.x,error)
        if error<.0005:break
    return best[1],best[2]


def run(a):
    out=a.output;out.mkdir(parents=True,exist_ok=True)
    sim=Simulation(a.scene,render=False)
    rows=[];plan=[]
    def collect(stage):
        forces={'left':0.,'right':0.};table=0;bad=[]
        for contact_id,c in enumerate(sim.data.contact):
            names=[sim.model.body(int(sim.model.geom_bodyid[g])).name for g in (c.geom1,c.geom2)]
            if 'tray' in names:
                force=np.zeros(6);mujoco.mj_contactForce(sim.model,sim.data,contact_id,force)
                for side in forces:
                    if any(n.startswith(side+'_gripper_finger') for n in names):forces[side]+=float(force[0])
                if 'table' in names:table+=1
            elif any(n.startswith(('left_','right_')) for n in names):bad.append(names)
        rows.append(dict(stage=stage,time=float(sim.data.time),qpos=sim.data.qpos.copy(),qvel=sim.data.qvel.copy(),warm=sim.data.qacc_warmstart.copy(),state=sim.state().copy(),action=sim.target.copy(),tray=sim.data.body('tray').xpos.copy(),forces=forces,table_contacts=table,bad_contacts=bad))
    def move(target,seconds,stage):
        start=sim.target.copy();steps=int(seconds*sim.fps)
        for i in range(steps):
            u=(i+1)/steps;u=u*u*(3-2*u)
            sim.target=start+(target-start)*u;collect(stage);sim.step()
        print(stage,'tray',sim.data.body('tray').xpos.tolist(),'grip',sim.state()[[7,15]].tolist(),flush=True)
    try:
        move(sim.target.copy(),1,'settle')
        print('home', {side:sim.tcp(side)[0].tolist() for side in ['left','right']},flush=True)
        home=sim.target.copy();rotation=lambda side: (Rotation.from_euler('y',a.pitch,degrees=True)*Rotation.from_euler('x',(-1 if side=='left' else 1)*a.tilt,degrees=True)*Rotation.from_euler('z',a.yaw,degrees=True)).as_matrix()
        for label,z,duration in [('approach',a.approach_z,4),('descend',a.grasp_z,3),('lift',a.grasp_z+a.lift_height,4)]:
            stage_x=(a.approach_x if label=="approach" and a.approach_x is not None else a.lift_x if label=="lift" and a.lift_x is not None else a.x)
            target=sim.target.copy()
            for side,sign,start in [('left',1,0),('right',-1,8)]:
                position=np.array([stage_x,sign*a.y,z]);q,error=solve(sim,side,position,rotation(side),target[start:start+7])
                plan.append(dict(stage=label,side=side,position=position.tolist(),q=q.tolist(),ik_error=float(error)))
                print(label,side,'IK',error,flush=True)
                if error>.025:raise RuntimeError('IK unreachable')
                target[start:start+7]=q
            if label=='lift':
                close=sim.target.copy();close[[7,15]]=a.opening;move(close,2,'close');target[[7,15]]=a.opening
            starts={side:sim.tcp(side)[0] for side in ['left','right']}
            rotations={side:Slerp([0,1],Rotation.from_matrix(np.stack([sim.tcp(side)[1],rotation(side)]))) for side in ['left','right']}
            count=int(duration*sim.fps)
            for i in range(count):
                if i%4==0:
                    block_start=sim.target.copy();block_end=target.copy()
                    u=min(1,(i+4)/count);u=u*u*(3-2*u)
                    for side,sign,start in [('left',1,0),('right',-1,8)]:
                        end=np.array([stage_x,sign*a.y,z]);pos=starts[side]+u*(end-starts[side])
                        q,error=solve(sim,side,pos,rotations[side](u).as_matrix(),block_start[start:start+7])
                        block_end[start:start+7]=q
                sim.target=block_start+(block_end-block_start)*((i%4+1)/4)
                collect(label);sim.step()
            print(label,'tray',sim.data.body('tray').xpos.tolist(),flush=True)
        move(sim.target.copy(),3,'hold')
    finally:
        report={'source':'program_generated_simulation','scene_sha256':hashlib.sha256(a.scene.read_bytes()).hexdigest(),'plan':plan,'params':{k:str(v) if isinstance(v,Path) else v for k,v in vars(a).items()}}
        if rows:
            held=[r for r in rows if r['stage']=='hold'];report.update(initial_z=float(rows[0]['tray'][2]),final_z=float(rows[-1]['tray'][2]),max_z=float(max(r['tray'][2] for r in rows)),hold_min_z=float(min([r['tray'][2] for r in held],default=0)),bad_contact_frames=sum(bool(r['bad_contacts']) for r in rows),last_forces=rows[-1]['forces'])
            report['success']=bool(held and report['hold_min_z']-report['initial_z']>.08 and all(r['table_contacts']==0 for r in held))
            np.savez_compressed(out/'trajectory.npz',**{key:np.array([r[key] for r in rows]) for key in ['time','qpos','qvel','warm','state','action','tray']},stage=np.array([r['stage'] for r in rows]))
            (out/'contacts.json').write_text(json.dumps([{k:v for k,v in r.items() if k in ('stage','time','forces','table_contacts','bad_contacts')} for r in rows]))
        (out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True);sim.close()

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--scene',type=Path,default=Path('outputs/r1pro_tray_scene/scene_physics.xml'));p.add_argument('--output',type=Path,required=True);p.add_argument('--x',type=float,default=.85);p.add_argument('--y',type=float,default=.205);p.add_argument('--grasp-z',type=float,default=.883);p.add_argument('--yaw',type=float,default=0);p.add_argument('--opening',type=float,default=.004);p.add_argument('--tilt',type=float,default=10);p.add_argument('--approach-z',type=float,default=1.13);p.add_argument('--lift-height',type=float,default=.15);p.add_argument('--pitch',type=float,default=0);p.add_argument('--approach-x',type=float);p.add_argument('--lift-x',type=float)
    run(p.parse_args())

