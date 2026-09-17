"""Versioned, deterministic airport tray scene generation; no controller changes."""
from __future__ import annotations
import argparse, copy, hashlib, json, math, os, shutil
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np

VERSION = "airport_trays/1.1.0"
TABLE = [1.2, .6, .8]  # long, short, top height
TRAY = [.56, .43, .105]
GAP = .015
TEMPLATE = Path("outputs/r1pro_tray_scene/scene_direct_grasp_720p.xml")

def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True)+"\n", encoding="utf-8")

def quat(yaw):
    return [math.cos(yaw/2), 0., 0., math.sin(yaw/2)]

def fmt(values):
    return " ".join(format(float(x), ".12g") for x in values)

def slots(gap=GAP):
    # Tray local long axis is perpendicular to belt long axis.
    return [{"slot_id":f"slot_{i}", "position_belt":[.005+TRAY[1]/2+i*(TRAY[1]+gap),TABLE[1]/2,0.],
             "yaw_belt":math.pi/2} for i in range(3)]

def belt_to_world(xy):
    return np.array([.55+xy[1], .6-xy[0]])

def corners(t):
    x,y=t["position_world"][:2];a=t["yaw_world"]
    r=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
    return np.array([[-.28,-.215],[-.28,.215],[.28,.215],[.28,-.215]])@r.T+[x,y]

def overlap(a,b,margin=.001):
    pa,pb=corners(a),corners(b)
    for p in (pa,pb):
        for edge in (p[1]-p[0],p[2]-p[1]):
            axis=np.array([-edge[1],edge[0]]);axis/=np.linalg.norm(axis)
            aa,bb=pa@axis,pb@axis
            if aa.max()+margin<=bb.min() or bb.max()+margin<=aa.min():return False
    return True

def supported_xy(xy,table):
    # Conservative inset avoids rounded tabletop corners.
    c=table["center_world"]
    return abs(xy[0]-c[0])<=.3-.015 and abs(xy[1]-c[1])<=.6-.015

def validate_spec(s):
    if s["schema_version"]!=VERSION:raise ValueError("schema_version")
    if len(s["tables"])!=3 or len(s["trays"])!=3:raise ValueError("object_count")
    if not 1<=s["aisle_gap_m"]<=2:raise ValueError("aisle_gap")
    if sum(t["support_table"]!="rear" for t in s["trays"])!=s["front_count"]:raise ValueError("front_count")
    for t in s["trays"]:
        table=next(x for x in s["tables"] if x["table_id"]==t["support_table"])
        if not supported_xy(t["position_world"][:2],table):raise ValueError("com_outside_table")
        if t["position_world"][2]<.8:raise ValueError("tray_below_table")
    if s["difficulty"]=="ordinary":
        for i,a in enumerate(s["trays"]):
            for b in s["trays"][i+1:]:
                if overlap(a,b):raise ValueError("ordinary_overlap")
    for slot in s["slots"]:
        x,y,z=slot["position_belt"]
        if not (.215<=x<=2.4-.215 and .28<=y<=.6-.28):raise ValueError("slot_outside_belt")
    if s["slot_gap_m"]<0:raise ValueError("negative_slot_gap")

def _sample_layout(seed,index,front_count,difficulty,split,attempt):
    rng=np.random.default_rng(np.random.SeedSequence([seed,attempt]))
    gap=float(rng.uniform(1,2))
    tables=[dict(table_id="front_left",center_world=[.85,0,0]),dict(table_id="front_right",center_world=[.85,-1.2,0]),dict(table_id="rear",center_world=[.25-gap,0,0])]
    for t in tables:t.update(dimensions_m=TABLE,yaw_world=math.pi/2)
    trays=[]
    # A stratified packing proposal permits 3 ordinary trays on the short rear table.
    rear_n=3-front_count
    for i in range(3):
        front=i<front_count
        if front:
            u=float((i+.5)*2.4/front_count+rng.uniform(-.12,.12));v=float(rng.uniform(.22,.38));xy=belt_to_world([u,v]);table=tables[0 if u<1.2 else 1]
            yaw=float(rng.uniform(-math.pi,math.pi))
        else:
            table=tables[2];j=i-front_count
            if rear_n==3 and difficulty=="ordinary":
                xy=np.array([table["center_world"][0]+rng.uniform(-.007,.007), (j-1)*.448+rng.uniform(-.002,.002)])
                yaw=float(rng.uniform(-.015,.015))
            else:
                rear_y=(j-(rear_n-1)/2)*.50+rng.uniform(-.02,.02)
                xy=np.array([table["center_world"][0]+rng.uniform(-.035,.035),rear_y])
                yaw=float(rng.uniform(-math.pi,math.pi))
        trays.append(dict(tray_id=f"tray_{i}",support_table=table["table_id"],position_world=[float(xy[0]),float(xy[1]),.801],yaw_world=yaw,roll=0.,pitch=0.,support_parent=table["table_id"]))
    if difficulty=="complex":
        pair=(0,1) if front_count>=2 else (front_count,front_count+1)
        low,high=[trays[j] for j in pair]
        if rear_n==3:
            low['position_world']=[tables[2]['center_world'][0],-.27,.801];low['yaw_world']=float(rng.uniform(-.08,.08))
            trays[2]['position_world']=[tables[2]['center_world'][0],.27,.801];trays[2]['yaw_world']=float(rng.uniform(-.08,.08))
        # Upright raised tray drops onto another tray; physics must confirm support.
        high.update(position_world=[low["position_world"][0]+float(rng.uniform(-.008,.008)),low["position_world"][1]+float(rng.uniform(-.008,.008)),.911],yaw_world=low["yaw_world"]+float(rng.uniform(-.025,.025)),support_table=low["support_table"],support_parent=low["tray_id"])
    night=bool(rng.integers(2));az=float(rng.uniform(-math.pi,math.pi));elev=float(rng.uniform(.55,1.35));strength=float(rng.uniform(.3,.55) if night else rng.uniform(.65,1.05))
    return dict(schema_version=VERSION,scene_id=f"airport_{index:04d}_{seed}",seed=seed,attempt=attempt,difficulty=difficulty,split=split,front_count=front_count,
      belt_frame=dict(origin_world=[.55,.6,.8],rotation_world_from_belt=[[0,1,0],[-1,0,0],[0,0,1]],axes="+X left to right; +Y into tabletop; +Z up"),
      aisle_gap_m=gap,aisle_gap_definition="nearest front/rear tabletop edges, world X",tables=tables,trays=trays,slots=slots(),slot_gap_m=GAP,
      robot=dict(base_position_world=[.55-gap*.5+float(rng.uniform(-.06,.06)),float(rng.uniform(-.18,.18)),0],base_yaw_world=float(rng.uniform(-math.pi,math.pi)),base_control_available=False,home_key="teleop_home",wrist_profile="direct_camera_front"),
      lighting=dict(mode="night" if night else "day",azimuth=az,elevation=elev,intensity=strength),camera=dict(width=1280,height=720),layout_kind="stacked" if difficulty=="complex" else "separate")

def sample(seed,index,front_count,difficulty,split,attempt):
    spec=_sample_layout(seed,index,front_count,difficulty,split,attempt)
    # Independent RNG preserves every prior layout and robot pose.
    rng=np.random.default_rng(np.random.SeedSequence([seed,attempt,110]))
    sources=[]
    for i,(x,y) in enumerate([(-1.8,-1.8),(1.4,-1.8),(-1.8,1.2),(1.4,1.2)]):
        intensity=float(rng.uniform(.15,.18))
        sources.append(dict(name=f'indoor_ceiling_{i}',position=[x,y,3.2],direction=[float(rng.uniform(-.08,.08)),float(rng.uniform(-.08,.08)),-1.],diffuse=[intensity,intensity*.98,intensity*.95],ambient=[.025]*3,castshadow=i==0))
    spec['lighting']=dict(mode='indoor_bright',profile_version='1.1',sources=sources,headlight_ambient=[.2]*3,headlight_diffuse=[.12]*3)
    return spec

def build_xml(spec, template, destination):
    root=ET.parse(template).getroot();w=root.find('worldbody')
    for a in root.find('asset'):
        if 'file' in a.attrib:a.set('file',os.path.relpath(template.parent/a.get('file'),destination.parent).replace('\\','/'))
    table=w.find("body[@name='table']");tray=w.find("body[@name='tray']");w.remove(table);w.remove(tray)
    for src,items,key in [(table,spec['tables'],'table_id'),(tray,spec['trays'],'tray_id')]:
        for item in items:
            b=copy.deepcopy(src);prefix=item[key]
            for e in b.iter():
                if 'name' in e.attrib:e.set('name',prefix+'__'+e.get('name'))
            b.set('name',prefix);b.set('pos',fmt(item.get('center_world',item.get('position_world'))));b.set('quat',fmt(quat(item['yaw_world'])))
            w.append(b)
    robot=w.find("body[@name='base_link']");robot.set('pos',fmt(spec['robot']['base_position_world']));robot.set('quat',fmt(quat(spec['robot']['base_yaw_world'])))
    # A world-mounted head camera follows the changed base transform at initialization.
    head=w.find("camera[@name='head']");p=np.fromstring(head.get('pos'),sep=' ');axes=np.fromstring(head.get('xyaxes'),sep=' ').reshape(2,3)
    yaw=spec['robot']['base_yaw_world'];r=np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]])
    head.set('pos',fmt(r@(p-[.5,0,0])+spec['robot']['base_position_world']));head.set('xyaxes',fmt((axes@r.T).ravel()))
    for light in list(w.findall('light')):w.remove(light)
    l=spec['lighting']
    for light in l['sources']:
        ET.SubElement(w,'light',name=light['name'],pos=fmt(light['position']),dir=fmt(light['direction']),directional='false',cutoff='85',exponent='0',attenuation='1 0 0',diffuse=fmt(light['diffuse']),ambient=fmt(light['ambient']),specular='.02 .02 .02',castshadow=str(light['castshadow']).lower())
    visual=root.find('visual');headlight=visual.find('headlight')
    if headlight is None:headlight=ET.SubElement(visual,'headlight')
    headlight.set('ambient',fmt(l['headlight_ambient']));headlight.set('diffuse',fmt(l['headlight_diffuse']));headlight.set('specular','0 0 0')
    # Same shared finish on visible tabletop and ground; collision friction unchanged.
    ET.SubElement(root.find('asset'),'material',name='airport_table_finish',rgba='0.8433891712 0.8266570506 0.7673756655 1',specular='.05',shininess='.1')
    floor=w.find("geom[@name='floor']");floor.set('material','airport_table_finish');floor.attrib.pop('rgba',None);floor.set('size','8 8 .05')
    for b in spec['tables']:
        e=w.find(f"body[@name='{b['table_id']}']/geom[@mesh='table_1']");e.set('material','airport_table_finish');e.attrib.pop('rgba',None)
    site=w.find("site[@name='place_target']")
    if site is not None:w.remove(site)
    for slot in spec['slots']:
        xy=belt_to_world(slot['position_belt'][:2]);ET.SubElement(w,'site',name=slot['slot_id'],pos=fmt([*xy,.802]),type='box',size='.28 .215 .001',rgba='.1 .8 .3 .2',group='4')
    key=root.find("keyframe/key[@name='teleop_home']");old=np.fromstring(key.get('qpos'),sep=' ')[:18]
    key.set('qpos',fmt(np.r_[old,np.concatenate([np.r_[t['position_world'],quat(t['yaw_world'])] for t in spec['trays']])]))
    ET.indent(root);return ET.tostring(root,encoding='unicode')

def physics_check(xml, path, spec):
    model=mujoco.MjModel.from_xml_path(str(path))
    d=mujoco.MjData(model);mujoco.mj_resetDataKeyframe(model,d,model.key('teleop_home').id)
    for a in range(model.nu):d.ctrl[a]=d.qpos[model.jnt_qposadr[model.actuator_trnid[a,0]]]
    mujoco.mj_forward(model,d)
    tray_ids={model.body(t['tray_id']).id:t for t in spec['trays']};table_ids={model.body(t['table_id']).id:t for t in spec['tables']}
    robot_root=model.body('base_link').id
    robots=set()
    for b in range(1,model.nbody):
        k=b
        while k and k!=robot_root:k=model.body_parentid[k]
        if k==robot_root:robots.add(b)
    # MuJoCo omits same-weld/static-static contacts: explicitly test robot/table hulls.
    robot_geoms=[g for g in range(model.ngeom) if model.geom_bodyid[g] in robots and model.geom_contype[g]]
    table_geoms=np.array([g for g in range(model.ngeom) if model.geom_bodyid[g] in table_ids and model.geom_contype[g]])
    for g in robot_geoms:
        distance=np.linalg.norm(d.geom_xpos[table_geoms]-d.geom_xpos[g],axis=1)
        nearby=table_geoms[distance<model.geom_rbound[table_geoms]+model.geom_rbound[g]+.001]
        for other in nearby:
            if mujoco.mj_geomDistance(model,d,g,int(other),.001,None)<-.0001:raise ValueError('robot_initial_collision')
    floor=model.geom('floor').id
    max_tilt=0.;max_pen=0.
    def inspect(initial=False):
        nonlocal max_tilt,max_pen
        for c in d.contact:
            a,b=[int(model.geom_bodyid[g]) for g in (c.geom1,c.geom2)]
            if a in robots or b in robots:
                if a in robots and b in robots or a in table_ids or b in table_ids or a in tray_ids or b in tray_ids:raise ValueError('robot_initial_collision' if initial else 'robot_collision')
            if a in tray_ids or b in tray_ids:
                if floor in (c.geom1,c.geom2):raise ValueError('tray_floor_contact')
                max_pen=max(max_pen,float(-c.dist))
                if c.dist<-.003:raise ValueError('illegal_penetration')
        for bid,t in tray_ids.items():
            body=d.body(bid);table=next(x for x in spec['tables'] if x['table_id']==t['support_table'])
            if not supported_xy(body.xipos[:2],table):raise ValueError('com_unsupported')
            tilt=math.degrees(math.acos(np.clip(body.xmat[8],-1,1)));max_tilt=max(max_tilt,tilt)
            if tilt>10:raise ValueError('tipped')
            if body.xpos[2]<.79:raise ValueError('slid_off_table')
    inspect(True)
    for step in range(round(2/model.opt.timestep)):
        mujoco.mj_step(model,d)
        if step%10==0:inspect()
    mujoco.mj_forward(model,d);inspect()
    # Contact graph proves support reaches a real tabletop even for stacked objects.
    edges={b:set() for b in tray_ids|table_ids}
    for c in d.contact:
        a,b=[int(model.geom_bodyid[g]) for g in (c.geom1,c.geom2)]
        if a in edges and b in edges and c.dist<.001:edges[a].add(b);edges[b].add(a)
    settled=[]
    for bid,t in tray_ids.items():
        body=d.body(bid);settled.append(dict(position_world=body.xpos.tolist(),yaw_world=math.atan2(body.xmat[3],body.xmat[0])))
    if spec['difficulty']=='ordinary':
        for i,t in enumerate(settled):
            if any(overlap(t,b,margin=0) for b in settled[i+1:]):raise ValueError('settled_ordinary_overlap')
    for bid,t in tray_ids.items():
        seen={bid};pending=[bid]
        while pending:
            for b in edges[pending.pop()]-seen:seen.add(b);pending.append(b)
        target=model.body(t['support_table']).id
        if target not in seen:raise ValueError('no_support_contact_chain')
        if np.linalg.norm(d.qvel[model.jnt_dofadr[model.body_jntadr[bid]]:model.jnt_dofadr[model.body_jntadr[bid]]+3])>.03:raise ValueError('not_settled')
    return dict(passed=True,settle_seconds=2,max_tilt_deg=max_tilt,max_penetration_m=max_pen,settled_trays=[dict(tray_id=t['tray_id'],position_world=d.body(b).xpos.tolist(),com_world=d.body(b).xipos.tolist()) for b,t in tray_ids.items()])

def generate_one(job):
    template=Path(job['template']);out=Path(job['out']);reject=[]
    for attempt in range(job['max_attempts']):
        s=sample(job['seed'],job['index'],job['front_count'],job['difficulty'],job['split'],attempt)
        directory=out/s['difficulty']/s['scene_id'];directory.mkdir(parents=True,exist_ok=True);path=directory/'scene.xml'
        try:
            validate_spec(s);xml=build_xml(s,template,path);candidate=path.with_name('candidate.xml');candidate.write_text(xml,encoding='utf-8');result=physics_check(xml,candidate,s);candidate.replace(path)
        except ValueError as e:
            reject.append(dict(attempt=attempt,reason=str(e)));continue
        s['runtime']={'mujoco':mujoco.__version__,'numpy':np.__version__};s['validation']=result;s['rejections']=reject;s['resample_count']=attempt;s['source_sha256']=hashlib.sha256(template.read_bytes()).hexdigest();s['xml_sha256']=hashlib.sha256(path.read_bytes()).hexdigest();s['xml']='scene.xml'
        write_json(directory/'scene.json',s)
        return dict(scene_id=s['scene_id'],seed=s['seed'],difficulty=s['difficulty'],front_count=s['front_count'],split=s['split'],resample_count=attempt,rejections=reject,json=(directory/'scene.json').relative_to(out).as_posix())
    write_json(directory/'failure.json',dict(rejections=reject,job=job))
    raise RuntimeError(f"{job['index']}: rejected {len(reject)} candidates: {Counter(r['reason'] for r in reject)}")

def generate_set(out,template,seed,count,workers,max_attempts,smoke):
    out.mkdir(parents=True,exist_ok=False)
    jobs=[]
    for i in range(count):
        fc=i%4;block=i//4
        complex_case=(i in (0,5)) if smoke else block in (0,21,42,88,99)
        jobs.append(dict(out=str(out.resolve()),template=str(template.resolve()),seed=seed+i,index=i,front_count=fc,difficulty='complex' if complex_case else 'ordinary',split='smoke' if smoke else ('train' if block%10<8 else 'validation' if block%10==8 else 'test'),max_attempts=max_attempts))
    results=[]
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for result in pool.map(generate_one,jobs):
            results.append(result);print(f"{out.name} {len(results)}/{count} {result['difficulty']} front={result['front_count']} retries={result['resample_count']}",flush=True)
    write_json(out/'manifest.json',dict(schema_version=VERSION,master_seed=seed,scenes=results,total=count,counts=dict(Counter(r['difficulty'] for r in results)),rejection_counts=dict(Counter(x['reason'] for r in results for x in r['rejections']))))
    return results

def previews(out,entries,limit=12):
    from PIL import Image,ImageDraw
    images=[]
    for entry in entries[:limit]:
        path=out/entry['json'];s=json.loads(path.read_text());m=mujoco.MjModel.from_xml_path(str(path.with_name('scene.xml')));d=mujoco.MjData(m);mujoco.mj_resetDataKeyframe(m,d,m.key('teleop_home').id);mujoco.mj_forward(m,d)
        camera=mujoco.MjvCamera();camera.lookat[:]=[-.1,-.5,.5];camera.distance=5.5;camera.azimuth=145;camera.elevation=-58
        with mujoco.Renderer(m,height=720,width=1280) as renderer:
            option=mujoco.MjvOption();option.geomgroup[3:]=0;renderer.update_scene(d,camera=camera,scene_option=option);im=Image.fromarray(renderer.render());im.save(path.with_name('preview.jpg'))
        im.thumbnail((426,240));draw=ImageDraw.Draw(im);draw.rectangle((0,0,426,26),fill='black');draw.text((5,6),f"front={s['front_count']} {s['difficulty']} {s['lighting']['mode']}",fill='white');images.append(im)
    sheet=Image.new('RGB',(426*3,240*math.ceil(len(images)/3)))
    for i,im in enumerate(images):sheet.paste(im,((i%3)*426,(i//3)*240))
    sheet.save(out/'preview_grid.jpg')

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--template',type=Path,default=TEMPLATE);p.add_argument('--seed',type=int,default=20260916);p.add_argument('--workers',type=int,default=4);p.add_argument('--max-attempts',type=int,default=250);p.add_argument('--smoke-only',action='store_true');a=p.parse_args()
    if a.output.exists():raise FileExistsError('Choose a new output directory')
    a.output.mkdir(parents=True)
    shutil.copyfile(Path(__file__).resolve().parents[1]/'schemas/airport_scene_v1_1.schema.json',a.output/'scene.schema.json')
    smoke=generate_set(a.output/'smoke',a.template,a.seed,12,a.workers,a.max_attempts,True);previews(a.output/'smoke',smoke)
    if not a.smoke_only:
        formal=generate_set(a.output/'formal',a.template,a.seed+10000,400,a.workers,a.max_attempts,False);previews(a.output/'formal',formal)

if __name__=='__main__':main()
