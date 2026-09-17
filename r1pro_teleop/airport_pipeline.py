"""Scene-grouped collection, metadata-only segment views, official LeRobot export."""
import json,hashlib,os,shutil,threading
from pathlib import Path
from collections import Counter,defaultdict
import xml.etree.ElementTree as ET
import numpy as np,h5py,mujoco
from .airport_annotations import annotate_arrays,CRITERIA,SKILLS,PICK_PHASES,PLACE_PHASES
from .airport_recording import AirportSimulation
from .dataset import export
from .core import CAMERAS
LOCK=threading.Lock()

def save(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_suffix(path.suffix+'.tmp');temp.write_text(json.dumps(value,indent=2,ensure_ascii=False),encoding='utf-8');temp.replace(path)

def split_map(ids,counts):
    if len(ids)!=sum(counts):raise ValueError('Split counts must cover every scene')
    ordered=sorted(ids);rng=np.random.default_rng(20260916);rng.shuffle(ordered)
    return {sid:split for split,part in zip(('train','val','test'),np.split(ordered,np.cumsum(counts)[:-1])) for sid in part.tolist()}

def init_collection(source,root):
    source=Path(source).resolve();root=Path(root).resolve();root.mkdir(parents=True,exist_ok=False)
    entries=json.loads((source/'smoke/manifest.json').read_text())['scenes'];splits=split_map([e['scene_id'] for e in entries],(8,2,2));registry=[]
    for e in entries:
        src=source/'smoke'/e['json'];spec=json.loads(src.read_text());dest=root/'scenes'/spec['scene_id'];dest.mkdir(parents=True)
        tree=ET.parse(src.with_name('scene.xml'))
        for node in tree.getroot().iter():
            if 'file' in node.attrib:node.set('file',os.path.relpath((src.parent/node.get('file')).resolve(),dest).replace('\\','/'))
        tree.write(dest/'scene.xml',encoding='unicode')
        spec['source_scene_xml_sha256']=spec['xml_sha256'];spec['xml_sha256']=hashlib.sha256((dest/'scene.xml').read_bytes()).hexdigest();spec['split']=splits[spec['scene_id']];save(dest/'scene.json',spec)
        registry.append(dict(scene_id=spec['scene_id'],split=spec['split'],json=f"scenes/{spec['scene_id']}/scene.json"))
    save(root/'scenes/index.json',registry);save(root/'splits/smoke.json',splits)
    formal=json.loads((source/'formal/manifest.json').read_text())['scenes'];save(root/'splits/formal_400_plan.json',split_map([e['scene_id'] for e in formal],(300,50,50)))
    save(root/'episodes/index.json',[]);save(root/'collection.json',dict(version='airport_collection/1.0',source='simulation',smoke_only=True,formal_collection_enabled=False,criteria=CRITERIA.__dict__,controller_sources=['human_teleoperation','program_generated'],privileged_exported_as_observation=False))
    for name in ('annotations','views','lerobot','quality'): (root/name).mkdir(exist_ok=True)
    return registry

def load_scene(root,scene_id):
    p=Path(root)/'scenes'/scene_id/'scene.json'
    spec=json.loads(p.read_text());xml=p.with_name(spec.get('scene_file','scene.xml'))
    if hashlib.sha256(xml.read_bytes()).hexdigest()!=spec['xml_sha256']:raise ValueError('Scene XML checksum mismatch')
    return spec,xml

def arrays(f):
    names={'pose':'tray_pose','forces':'gripper_contact_force','table':'table_contact','support':'support_contact','floor':'floor_contact','bottom':'min_z','illegal':'illegal_collision','penetration':'max_penetration','tcp':'tcp'}
    return {**{key:f['privileged/'+value][:] for key,value in names.items()},'state':f['state'][:],**({'base_pose':f['base_state'][:]} if 'base_state' in f else {})}

def raw_quality(f):
    n=int(f.attrs['num_frames']);fps=int(f.attrs['fps']);issues=[];camera={};nan=[]
    for name in ('state','action','qpos','qvel','timestamp')+tuple(k for k in ('base_state','base_action') if k in f):
        if not np.isfinite(f[name][:]).all():nan.append(name)
    timestamps=bool(np.allclose(f['timestamp'][:],np.arange(n)/fps,atol=1e-6))
    action=f['action'][:];arm=[0,1,2,3,4,5,6,8,9,10,11,12,13,14]
    jump=float(np.max(abs(np.diff(action[:,arm],axis=0)))) if n>1 else 0.
    for c in CAMERAS:
        ds=f['images/'+c];blank=sum(float(ds[i].std())<2 for i in range(n));camera[c]=dict(shape=list(ds.shape[1:]),blank_frames=blank)
        if tuple(ds.shape[1:])!=(720,1280,3):issues.append('wrong_resolution:'+c)
        if blank:issues.append('blank_frames:'+c)
    if fps!=20:issues.append('fps_not_20')
    if nan:issues.append('nonfinite')
    if not timestamps:issues.append('irregular_timestamps')
    if jump>.1:issues.append('action_jump')
    if n<2:issues.append('too_short')
    if bool(f.attrs.get('base_control_available',False)):
        if 'base_action' not in f or f['base_action'].shape!=(n,3):issues.append('missing_base_action')
        elif n>1 and np.max(abs(np.diff(f['base_action'][:],axis=0)))>.025:issues.append('base_action_jump')
    age=f['input_age_s'][:];source=str(f.attrs['controller_source'])
    stale=float(np.mean((age<0)|(age>.25)))
    elapsed=float(f['wall_monotonic'][-1]-f['wall_monotonic'][0]);realtime_ratio=((n-1)/fps)/max(elapsed,1e-9)
    if source=='human_teleoperation' and stale>.1:issues.append('stale_controller_input')
    if source=='human_teleoperation' and realtime_ratio<.8:issues.append('below_realtime_capture')
    return dict(frames=n,fps=fps,cameras=camera,nan_fields=nan,timestamps_regular=timestamps,max_arm_action_jump=jump,stale_input_fraction=stale,realtime_ratio=realtime_ratio,issues=issues,valid=not issues)

def replay(path,scene):
    sim=AirportSimulation(scene,render=False);errors=[]
    try:
        with h5py.File(path) as f:
            sim.data.qpos[:]=f['qpos'][0];sim.data.qvel[:]=f['qvel'][0];sim.data.qacc_warmstart[:]=f['qacc_warmstart'][0];sim.data.time=float(f.attrs['initial_sim_time']);mujoco.mj_forward(sim.model,sim.data)
            for i,target in enumerate(f['action']):
                errors.append(float(np.max(abs(sim.data.qpos-f['qpos'][i]))));sim.target=target.copy()
                if 'base_action' in f:sim.base_target=f['base_action'][i].astype(float)
                sim.step()
    finally:sim.close()
    return dict(passed=max(errors,default=1)<1e-5,max_qpos_error=max(errors,default=1),frames=len(errors))

def finish_episode(root,path,export_video=True):
    root=Path(root);path=Path(path)
    with h5py.File(path) as f:
        spec=json.loads(f.attrs['scene_json']);sid=spec['scene_id'];eid=path.stem;source=str(f.attrs['controller_source']);n=int(f.attrs['num_frames']);base_available=bool(f.attrs.get('base_control_available',False))
        if n<2:raise ValueError('Episode too short; raw retained')
        ann=annotate_arrays(arrays(f),spec,eid,source,str(f.attrs['operator_result']),int(f.attrs['fps']));quality=raw_quality(f)
        if f.attrs.get('agent_policy') in ('diagnostic_hold','base_motion_diagnostic'):
            ann['result']='needs_review';ann['success']=False
            for segment in ann['segments']:segment['success']=False;segment['needs_review']=True;segment['failure_reason']='diagnostic_only'
    _,scene=load_scene(root,sid);replayed=replay(path,scene)
    quality.update(replay=replayed,controller_source=source,environment_source='simulation',task_result=ann['result'],needs_review=ann['result']=='needs_review',scene_id=sid,episode_id=eid,training_candidate=bool(quality['valid'] and replayed['passed'] and ann['success']))
    with h5py.File(path,'r+') as f:f.attrs['result']='success' if ann['success'] else 'failure' if ann['result']=='failure' else 'unlabeled';f.attrs['success_operator']=source=='human_teleoperation' and ann['operator_result']=='success'
    save(root/'annotations'/eid/'events.json',ann['events']);save(root/'annotations'/eid/'segments.json',ann['segments']);save(root/'annotations'/eid/'outcome.json',{k:v for k,v in ann.items() if k not in ('events','segments')})
    dataset=root/'lerobot'/eid
    if export_video and quality['valid'] and replayed['passed']:
        export(path.parent,dataset,'local/'+eid,include_all=True)
        manifest=json.loads((dataset/'source_manifest.json').read_text());manifest.update(scene_id=sid,split=spec['split'],privileged_observations_exported=False,training_candidate=quality['training_candidate'],split_guidance='Use collection scene_id split; never split frames or segments independently');save(dataset/'source_manifest.json',manifest)
        quality['lerobot_loader_first_last']=True
    else:quality['lerobot_loader_first_last']=False
    save(root/'quality'/f'{eid}.json',quality)
    entry=dict(episode_id=eid,scene_id=sid,split=spec['split'],raw=path.relative_to(root).as_posix(),lerobot=dataset.relative_to(root).as_posix() if dataset.exists() else None,controller_source=source,environment_source='simulation',result=ann['result'],frames=n,fps=20,difficulty=spec['difficulty'],light=spec['lighting']['mode'],front_count=spec['front_count'],base_control_available=base_available,training_candidate=quality['training_candidate'])
    with LOCK:
        entries=json.loads((root/'episodes/index.json').read_text())
        if any(x['episode_id']==eid for x in entries):raise ValueError('Episode already finalized')
        entries.append(entry);save(root/'episodes/index.json',entries);aggregate(root,entries)
    return entry

def aggregate(root,entries):
    root=Path(root);splits=json.loads((root/'splits/smoke.json').read_text());leaks=[];views={x:[] for x in ('adjust','pick','place','full_arrangement')};counts={'skill':{k:dict(count=0,duration_s=0.) for k in SKILLS},'phase':{k:dict(count=0,duration_s=0.) for k in sorted(set(PICK_PHASES+PLACE_PHASES+['adjust','recover','unknown','verified']))}};results=Counter({k:0 for k in ('success','failure','aborted','needs_review')});sources=Counter({'human_teleoperation':0,'program_generated':0});recoveries=0;review_segments=0
    def add(dimension,key,seconds):
        group=counts.setdefault(dimension,{});row=group.setdefault(str(key),dict(count=0,duration_s=0.));row['count']+=1;row['duration_s']+=seconds
    for e in entries:
        if e['split']!=splits[e['scene_id']]:leaks.append(e['episode_id'])
        results[e['result']]+=1;sources[e['controller_source']]+=1
        for dim in ('difficulty','light','front_count','controller_source'):add(dim,e[dim],e['frames']/e['fps'])
        segments=json.loads((root/'annotations'/e['episode_id']/'segments.json').read_text())
        for s in segments:
            if not 0<=s['start_frame']<s['end_frame']<=e['frames']:raise ValueError('Invalid segment range')
            if s['scene_id']!=e['scene_id']:leaks.append(s['segment_id'])
            duration=(s['end_frame']-s['start_frame'])/e['fps'];add('skill',s['skill'],duration);add('phase',s['phase'],duration);add('source_skill',e['controller_source']+':'+s['skill'],duration)
            recoveries+=int(s['recovery']);review_segments+=int(s['needs_review'])
            if s['primary_class'] in views:views[s['primary_class']].append(dict(**s,split=e['split'],raw=e['raw'],lerobot=e['lerobot'],training_candidate=e['training_candidate'],quality_report='quality/'+e['episode_id']+'.json'))
        views['full_arrangement'].append(dict(**e,start_frame=0,end_frame=e['frames']))
    for name,items in views.items():save(root/'views'/f'{name}.json',items)
    invalid=Counter()
    for specpath in (root/'scenes').glob('*/scene.json'):
        for r in json.loads(specpath.read_text()).get('rejections',[]):invalid[r['reason']]+=1
    quality=[json.loads((root/'quality'/f"{e['episode_id']}.json").read_text()) for e in entries]
    save(root/'quality/summary.json',dict(episodes=len(entries),counts=counts,results=dict(results),controller_sources=dict(sources),environment_sources={'simulation':len(entries),'real_robot':0},recoveries=recoveries,needs_review_segments=review_segments,scene_leakage=leaks,invalid_scene_candidate_reasons=dict(invalid),episode_quality=quality,segment_views_copy_video=False,base_control_available=any(e.get("base_control_available",False) for e in entries)))
