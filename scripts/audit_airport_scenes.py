"""Audit exported scene metadata, hashes, deterministic sampling, and coverage."""
import argparse, hashlib, json, sys
from pathlib import Path
from collections import Counter
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from r1pro_teleop.airport_scenes import sample, validate_spec, build_xml, TEMPLATE

def main():
 p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args();reports={}
 for name,count in [('smoke',12),('formal',400)]:
  root=a.root/name;manifest=json.loads((root/'manifest.json').read_text());assert manifest['total']==count
  scenes=[]
  for row in manifest['scenes']:
   path=root/row['json'];s=json.loads(path.read_text());validate_spec(s)
   index=int(s['scene_id'].split('_')[1]);reproduced=sample(s['seed'],index,s['front_count'],s['difficulty'],s['split'],s['attempt'])
   assert all(s[k]==v for k,v in reproduced.items()),s['scene_id']
   xml=build_xml(reproduced,TEMPLATE.resolve(),path.with_name('scene.xml').resolve())
   assert xml==path.with_name('scene.xml').read_text(encoding='utf-8'),s['scene_id']
   assert hashlib.sha256(path.with_name('scene.xml').read_bytes()).hexdigest()==s['xml_sha256']
   assert s['validation']['passed'] and s['validation']['settle_seconds']==2
   assert s['resample_count']==len(s['rejections'])
   scenes.append(s)
  assert Counter(s['front_count'] for s in scenes)=={i:count//4 for i in range(4)}
  assert sum(s['difficulty']=='complex' for s in scenes)==(2 if name=='smoke' else 20)
  reports[name]={'count':count,'front_counts':dict(Counter(s['front_count'] for s in scenes)),'difficulty':dict(Counter(s['difficulty'] for s in scenes)),'splits':dict(Counter(s['split'] for s in scenes)),'lighting':dict(Counter(s['lighting']['mode'] for s in scenes)),'max_resamples':max(s['resample_count'] for s in scenes),'total_rejections':sum(s['resample_count'] for s in scenes),'rejection_counts':manifest['rejection_counts'],'deterministic_json_and_xml':True,'all_two_second_physics_checks_passed':True,'max_tilt_deg':max(s['validation']['max_tilt_deg'] for s in scenes)}
 print(json.dumps(reports,indent=2));(a.root/'audit.json').write_text(json.dumps(reports,indent=2))
if __name__=='__main__':main()
