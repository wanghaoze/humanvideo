"""Headless IK reachability probe; no contacts, task execution, or real robot control."""
import argparse
import json
from pathlib import Path
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
from .core import Simulation


def run(scene):
    sim=Simulation(scene,render=False)
    results=[]
    try:
        for side,sign,start in [('left',1,0),('right',-1,8)]:
            tx,ty,tz=sim.model.body('tray').pos
            gx,gy,gz=sim.model.site('place_target').pos
            for label,x,y,z in [('pickup_side',tx,ty+sign*.205,tz+.099),
                                ('pickup_front',tx-.26,ty+sign*.12,tz+.099),
                                ('place_side',gx,gy+sign*.205,gz+.098),
                                ('place_front',gx-.26,gy+sign*.12,gz+.098)]:
                sim.reset()
                home_tcp,rotation=sim.tcp(side)
                shoulder=sim.data.xanchor[sim.arms[side][0]].copy()
                chain=np.vstack([sim.data.xanchor[sim.arms[side]],home_tcp])
                reach_bound=float(np.linalg.norm(np.diff(chain,axis=0),axis=1).sum())
                target=np.array([x,y,z])
                for _ in range(150):
                    sim.target[start:start+7]=sim.ik(side,target,rotation)
                d=sim.ik_data
                for i,j in enumerate(sim.arms[side]):
                    d.qpos[sim.model.jnt_qposadr[j]]=sim.target[start+i]
                mujoco.mj_forward(sim.model,d)
                site=d.site(side+'_tcp')
                distance=float(np.linalg.norm(site.xpos-target))
                angle=float(np.degrees(np.linalg.norm(Rotation.from_matrix(
                    rotation@site.xmat.reshape(3,3).T).as_rotvec())))
                qids=[sim.model.jnt_qposadr[j] for j in sim.arms[side]]
                limits=sim.model.jnt_range[sim.arms[side]]
                rng=np.random.default_rng(42)
                def residual(q):
                    d.qpos[qids]=q
                    mujoco.mj_kinematics(sim.model,d)
                    s=d.site(side+'_tcp')
                    return np.r_[s.xpos-target, .15*Rotation.from_matrix(
                        rotation@s.xmat.reshape(3,3).T).as_rotvec()]
                solutions=[]
                for seed in [sim.target[start:start+7].copy()]+[
                        rng.uniform(limits[:,0]+.02,limits[:,1]-.02) for _ in range(5)]:
                    fit=least_squares(residual,seed,bounds=(limits[:,0]+.001,limits[:,1]-.001),max_nfev=150)
                    e=residual(fit.x)
                    solutions.append((np.linalg.norm(e),float(np.linalg.norm(e[:3])),
                                      float(np.degrees(np.linalg.norm(e[3:])/.15)),fit.x.tolist()))
                best=min(solutions)
                free=least_squares(lambda q:residual(q)[:3],best[3],
                    bounds=(limits[:,0]+.001,limits[:,1]-.001),max_nfev=200)
                free_error=float(np.linalg.norm(residual(free.x)[:3]))
                results.append({'side':side,'waypoint':label,'target_m':target.tolist(),
                                'shoulder_m':shoulder.tolist(),'chain_length_bound_m':reach_bound,
                                'shoulder_target_distance_m':float(np.linalg.norm(target-shoulder)),
                                'position_error_m':distance,'orientation_error_deg':angle,
                                'reachable_with_current_ik':distance<.02 and angle<10,
                                'multistart_position_error_m':best[1],
                                'multistart_orientation_error_deg':best[2],
                                'multistart_reachable':best[1]<.02 and best[2]<10,
                                'free_orientation_position_error_m':free_error,
                                'multistart_joint_solution':best[3]})
        return {'waypoints':results,'note':'Keeps reset TCP orientation; no collision check or grasp planning. Failure is not proof that all IK solutions are impossible.'}
    finally:sim.close()


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--scene',type=Path,default=Path('outputs/r1pro_tray_scene/scene_physics.xml'))
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();r=run(a.scene);a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
