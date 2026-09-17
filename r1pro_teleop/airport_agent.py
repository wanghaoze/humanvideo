"""Callable simulation collection agent. Does not impersonate a human or navigate a fixed base."""
import argparse,json,uuid,time,os,subprocess
from pathlib import Path
import numpy as np
from .airport_pipeline import init_collection,load_scene,finish_episode,save
from .airport_recording import AirportSimulation,AirportRecorder

def run_queue(root,scene_ids,seconds=3,action_file=None):
    root=Path(root).resolve()
    if not 1<=len(scene_ids)<=3:raise ValueError('This baseline only permits 1-3 smoke scenes; no formal batch capture')
    registry={r['scene_id'] for r in json.loads((root/'scenes/index.json').read_text())}
    if not set(scene_ids)<=registry:raise ValueError('Only registered smoke scenes are allowed')
    results=[]
    for sid in scene_ids:
        spec,xml=load_scene(root,sid);sim=AirportSimulation(xml,width=1280,height=720)
        home=sim.data.qpos.copy();initial_match=all(np.allclose(sim.data.body(t['tray_id']).xpos,t['position_world'],atol=1e-8) for t in spec['trays'])
        directory=root/'episodes'/('agent_'+uuid.uuid4().hex)
        if action_file:
            data=np.load(action_file,allow_pickle=False)
            if str(data['scene_id'])!=sid or str(data['scene_sha256'])!=spec['xml_sha256']:raise ValueError('Action plan must be validated against exact scene')
            actions=data['action']
            if actions.ndim!=2 or actions.shape[1]!=16 or not np.isfinite(actions).all():raise ValueError('Invalid action plan')
            if np.max(abs(np.diff(np.vstack([sim.target,actions]),axis=0)))>.06:raise ValueError('Unsafe action step')
            reason=None
        else:
            # Without a validated controller or mobile base, record only a clearly
            # labelled diagnostic hold; NEVER teleport robot/trays to fake a task.
            actions=np.repeat(sim.target[None],max(2,round(seconds*20)),axis=0)
            reason='base_control_unavailable_and_no_validated_arrangement_policy'
        rec=AirportRecorder(directory,sim,spec,'program_generated')
        rec.file.attrs['agent_policy']='validated_joint_plan' if action_file else 'diagnostic_hold'
        rec.file.attrs['agent_blocker']=reason or ''
        try:
            for action in actions:
                sim.target=np.asarray(action,np.float32);rec.append(sim,sim.images(),-1)
                evidence=rec.evidence.row()
                if evidence['privileged/illegal_collision'] or evidence['privileged/max_penetration']>.003:
                    reason='collision_guard';break
                forward=sim.workspace_rotation[:,0];side_axis=sim.workspace_rotation[:,1]
                flipped=any(sim.tcp(side)[1][:,1]@side_axis<=0 for side in ('left','right'))
                grasp=np.any(np.all(evidence['privileged/gripper_contact_force']>.5,axis=1))
                behind=grasp and any((sim.data.body(side+'_realsense_link').xpos-sim.tcp(side)[0])@forward<=0 for side in ('left','right'))
                if flipped or behind:reason='wrist_camera_guard';break
                sim.step()
            path=rec.finish('unlabeled' if reason else 'success')
        finally:sim.close()
        result=finish_episode(root,path);result.update(agent_status='blocked' if reason else 'completed' if result['result']=='success' else 'needs_review',blocker=reason,initial_scene_matches_json=initial_match)
        results.append(result);save(root/'agent/latest_queue.json',results);print(json.dumps(result),flush=True)
    return results

def run_base_smoke(root,scene_ids):
    """Agent controls the real simulated base DOFs; diagnostic, not task success."""
    from .airport_mobile import move_base
    root=Path(root);results=[]
    registry={r['scene_id'] for r in json.loads((root/'scenes/index.json').read_text())}
    if not 1<=len(scene_ids)<=3 or not set(scene_ids)<=registry:raise ValueError('Only 1-3 registered smoke scenes')
    for sid in scene_ids:
        spec,xml=load_scene(root,sid);sim=AirportSimulation(xml,width=1280,height=720)
        rec=AirportRecorder(root/'episodes'/('agent_base_'+uuid.uuid4().hex),sim,spec,'program_generated')
        rec.file.attrs['agent_policy']='base_motion_diagnostic'
        rec.file.attrs['task_attempted']=False
        home=sim.base_state().copy();checks=[];reason=None
        def emit(skill):
            rec.append(sim,sim.images(),-1,{'agent/commanded_skill':np.int32(0)})
        try:
            # Bounded movement in the initial aisle, then return to the same pose.
            for dx,dy,dyaw in [(0.,-.1,0.),(0.,-.1,.12),(0.,0.,0.)]:
                checks.append(move_base(sim,home+np.array([dx,dy,dyaw]),emit))
        except RuntimeError as error:reason=str(error)
        finally:
            rec.file.attrs['agent_blocker']=reason or 'base_only_diagnostic_not_arrangement'
            path=rec.finish('failure' if reason else 'unlabeled');sim.close()
        result=finish_episode(root,path)
        result.update(base_checks=checks,base_test_passed=not reason,blocker=reason,arrangement_attempted=False)
        results.append(result);save(root/'agent/base_checks.json',results);print(json.dumps(result),flush=True)
    return results

def main():
    p=argparse.ArgumentParser();sub=p.add_subparsers(dest='command',required=True)
    init=sub.add_parser('init');init.add_argument('--source',type=Path,required=True);init.add_argument('--root',type=Path,required=True);init.add_argument('--mobile-base',action='store_true')
    for name in ('run-smoke','run-queue'):
        cmd=sub.add_parser(name);cmd.add_argument('--root',type=Path,required=True);cmd.add_argument('--scene-ids',nargs='+');cmd.add_argument('--seconds',type=float,default=3);cmd.add_argument('--action-file',type=Path)
    base=sub.add_parser('run-base-smoke');base.add_argument('--root',type=Path,required=True);base.add_argument('--scene-ids',nargs='+')
    task=sub.add_parser('run-arrangement');task.add_argument('--root',type=Path,required=True);task.add_argument('--scene-id',default='airport_0009_20260925')
    serve=sub.add_parser('serve');serve.add_argument('--root',type=Path,required=True);serve.add_argument('--scene-id',required=True);serve.add_argument('--port',type=int,default=8767);serve.add_argument('--token-file',type=Path,default=Path('runs/server.token'))
    status=sub.add_parser('status');status.add_argument('--root',type=Path,required=True)
    a=p.parse_args()
    if a.command=='init':
        print(json.dumps(init_collection(a.source,a.root),indent=2))
        if a.mobile_base:
            from .airport_mobile import enable_collection
            enable_collection(a.root)
    elif a.command=='run-base-smoke':
        registry=json.loads((a.root/'scenes/index.json').read_text())
        run_base_smoke(a.root,a.scene_ids or [registry[i]['scene_id'] for i in (1,2,3)])
    elif a.command in ('run-smoke','run-queue'):
        if not 0.1<=a.seconds<=30:raise ValueError('Bounded diagnostic duration is 0.1-30 seconds')
        registry=json.loads((a.root/'scenes/index.json').read_text());ids=a.scene_ids or [registry[i]['scene_id'] for i in (1,2,3)];run_queue(a.root,ids,a.seconds,a.action_file)
    elif a.command=='run-arrangement':
        import sys
        from .airport_arrange import Controller
        if a.scene_id!='airport_0009_20260925':raise ValueError('The current bounded task policy is validated only on smoke scene 0009')
        spec,xml=load_scene(a.root,a.scene_id)
        if not spec.get('base_control_available') or spec.get('yaw_period_rad')!=float(np.pi):raise ValueError('Requires mobile scene and explicit rectangular long-axis orientation contract')
        directory=a.root/('arrangement_'+uuid.uuid4().hex);controller=Controller(xml,directory);error=None
        try:controller.run()
        except Exception as exc:error=str(exc)
        finally:report=controller.save(error)
        if error:raise RuntimeError('Task stopped; complete attempt evidence retained at '+str(directory)+': '+error)
        script=Path(__file__).resolve().parents[1]/'scripts/export_arrangement_attempt.py'
        subprocess.run([sys.executable,str(script),'--root',str(a.root),'--attempt',str(directory),'--scene-id',a.scene_id],check=True)
    elif a.command=='serve':
        from .airport_service import AirportService
        from .server import create_app
        import uvicorn
        uvicorn.run(create_app(AirportService(a.root,a.scene_id),a.token_file.read_text().strip()),host='0.0.0.0',port=a.port,log_level='warning')
    elif a.command=='status':
        for rel in ('agent/latest_queue.json','quality/summary.json'):
            path=a.root/rel
            if path.exists():print(path.read_text())
if __name__=='__main__':main()
