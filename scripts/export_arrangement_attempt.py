"""Replay agent actions, validate task physics and render/export the entire episode."""
import argparse,json,sys,uuid,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np,mujoco,av
from PIL import Image,ImageDraw
from r1pro_teleop.airport_recording import AirportSimulation,AirportRecorder
from r1pro_teleop.airport_pipeline import load_scene,finish_episode,save
from r1pro_teleop.airport_annotations import annotate_arrays,orientation


def evidence_arrays(d):
    mapping={'pose':'tray_pose','forces':'gripper_contact_force','table':'table_contact','support':'support_contact','floor':'floor_contact','bottom':'min_z','illegal':'illegal_collision','penetration':'max_penetration','tcp':'tcp','state':'state','base_pose':'base_state'}
    return {k:d[v] for k,v in mapping.items()}


def validate(d,spec):
    ann=annotate_arrays(evidence_arrays(d),spec,'validation','program_generated')
    ids=[0,2,1];pos=d['tray_pose'][-1,ids,:3];yaw,tilt=orientation(d['tray_pose'][-1,ids,3:])
    r=np.array(spec['belt_frame']['rotation_world_from_belt']);origin=np.array(spec['belt_frame']['origin_world']);goal=np.array([s['position_belt'] for s in spec['slots']])@r.T+origin
    extent=.28*abs(np.sin(yaw))+.215*abs(np.cos(yaw));gaps=pos[:-1,1]-pos[1:,1]-extent[:-1]-extent[1:]
    return dict(success=ann['success'],result=ann['result'],frames=len(d['time']),duration_s=len(d['time'])/20,position_errors_m=np.linalg.norm(pos-goal,axis=1).tolist(),long_axis_errors_deg=np.rad2deg(abs(np.arctan2(np.sin(2*yaw),np.cos(2*yaw))/2)).tolist(),tilt_deg=np.rad2deg(tilt).tolist(),gaps_m=gaps.tolist(),illegal_contact_frames=int(np.count_nonzero(d['illegal_collision'])),floor_contact_frames=int(np.count_nonzero(d['floor_contact'])),max_penetration_m=float(d['max_penetration'].max()),controller_source='program_generated',environment_source='simulation',orientation_contract='Rectangular tray long axis: 180 degrees equivalent; no marked front orientation required')


def main():
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--attempt',type=Path,required=True);p.add_argument('--scene-id',default='airport_0009_20260925');p.add_argument('--validate-only',action='store_true');a=p.parse_args()
    root=a.root.resolve();spec,scene=load_scene(root,a.scene_id);loaded=np.load(a.attempt/'trajectory.npz',allow_pickle=False);d={k:loaded[k] for k in loaded.files};loaded.close()
    report=validate(d,spec)
    execution=json.loads((a.attempt/'report.json').read_text())
    if execution.get('error') or not execution.get('execution_completed'):
        report.update(success=False,result='failure',execution_error=execution.get('error'))
    save(a.attempt/'acceptance.json',report);print(json.dumps(report),flush=True)
    if a.validate_only:return
    if not report['success']:raise RuntimeError('Task did not pass; no successful task export')
    s=AirportSimulation(scene,width=1280,height=720);directory=root/'episodes'/('arrangement_'+uuid.uuid4().hex);rec=AirportRecorder(directory,s,spec,'program_generated')
    rec.file.attrs['agent_policy']='airport_arrangement_script_v1';rec.file.attrs['agent_uses_privileged_state']=True
    full=av.open(str(a.attempt/'full_arrangement_720p.mp4'),'w');fs=full.add_stream('libx264',rate=20);fs.width=1280;fs.height=720;fs.pix_fmt='yuv420p';fs.options={'crf':'18','preset':'fast'}
    preview=av.open(str(a.attempt/'preview_4x_720p.mp4'),'w');ps=preview.add_stream('libx264',rate=20);ps.width=1280;ps.height=720;ps.pix_fmt='yuv420p';ps.options={'crf':'18','preset':'fast'}
    max_error=0.;wrist_error_frames=0;start=time.monotonic()
    try:
        for i in range(len(d['time'])):
            error=float(np.max(abs(s.data.qpos-d['qpos'][i])));max_error=max(error,max_error)
            if error>1e-5:raise RuntimeError(f'Action replay diverged at frame {i}: {error}')
            s.target=d['action'][i].astype(np.float32);s.base_target=d['base_action'][i].astype(np.float32)
            rec.append(s,s.images(),-1,{'agent/target_tray':np.int32(d['target_tray'][i])})
            contact=(d['gripper_contact_force'][i]>.5).any()
            if contact:
                forward=s.workspace_rotation[:,0];sideaxis=s.workspace_rotation[:,1]
                if any((s.data.body(side+'_realsense_link').xpos-s.tcp(side)[0])@forward<=0 or s.tcp(side)[1][:,1]@sideaxis<=0 for j,side in enumerate(('left','right')) if (d['gripper_contact_force'][i,:,j]>.5).any()):wrist_error_frames+=1
            im=Image.fromarray(s.image('overview'));draw=ImageDraw.Draw(im);draw.rectangle((0,0,1280,30),fill='#172330');draw.text((12,9),f'PROGRAM GENERATED | SIMULATION | {i/20:6.1f}s | {str(d["stage"][i])} | tray {d["target_tray"][i]}',fill='white')
            rgb=np.asarray(im);frame=av.VideoFrame.from_ndarray(rgb,format='rgb24')
            for packet in fs.encode(frame):full.mux(packet)
            if i%4==0:
                for packet in ps.encode(av.VideoFrame.from_ndarray(rgb,format='rgb24')):preview.mux(packet)
            if i in (0,len(d['time'])-1):im.save(a.attempt/('initial.png' if i==0 else 'final.png'))
            s.step()
            if i%200==0:print(json.dumps(dict(render_frame=i,total=len(d['time']),elapsed_s=round(time.monotonic()-start))),flush=True)
        for packet in fs.encode():full.mux(packet)
        for packet in ps.encode():preview.mux(packet)
        path=rec.finish('unlabeled')
    finally:full.close();preview.close();s.close()
    report.update(replay_max_qpos_error=max_error,wrist_camera_violation_frames=wrist_error_frames,raw_episode=str(path));save(a.attempt/'acceptance.json',report)
    if wrist_error_frames:raise RuntimeError('Wrist camera orientation guard failed')
    entry=finish_episode(root,path);save(a.attempt/'episode.json',entry);print(json.dumps(entry),flush=True)

if __name__=='__main__':main()
