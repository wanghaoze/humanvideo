"""Upgrade verified layouts to bright indoor lights, proving physics XML unchanged."""
import argparse,copy,hashlib,json,sys,shutil
from pathlib import Path
import xml.etree.ElementTree as ET
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from r1pro_teleop.airport_scenes import sample,build_xml,previews,write_json,TEMPLATE,VERSION

def physics_tree(xml):
 root=ET.fromstring(xml)
 for parent in root.iter():
  for child in list(parent):
   if child.tag in ('light','headlight'):parent.remove(child)
 def canonical(e):return (e.tag,tuple(sorted(e.attrib.items())),tuple(canonical(c) for c in e))
 return canonical(root)

def main():
 p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 a.output.mkdir(parents=True,exist_ok=False)
 shutil.copyfile('schemas/airport_scene_v1_1.schema.json',a.output/'scene.schema.json')
 for batch in ('smoke','formal'):
  manifest=json.loads((a.source/batch/'manifest.json').read_text());manifest['schema_version']=VERSION
  for entry in manifest['scenes']:
   src=a.source/batch/entry['json'];dest=a.output/batch/entry['json'];dest.parent.mkdir(parents=True,exist_ok=False)
   old=json.loads(src.read_text());index=int(old['scene_id'].split('_')[1])
   new=sample(old['seed'],index,old['front_count'],old['difficulty'],old['split'],old['attempt'])
   for key in new:
    if key not in ('schema_version','lighting'):assert old[key]==new[key],key
   xml=build_xml(new,TEMPLATE.resolve(),dest.with_name('scene.xml').resolve())
   assert physics_tree(src.with_name('scene.xml').read_text())==physics_tree(xml),old['scene_id']
   dest.with_name('scene.xml').write_text(xml,encoding='utf-8')
   old.update(new);old['xml_sha256']=hashlib.sha256(dest.with_name('scene.xml').read_bytes()).hexdigest()
   old['lighting_migration']={'physics_xml_identical':True,'source_schema':'airport_trays/1.0.0','source_xml_sha256':json.loads(src.read_text())['xml_sha256'],'validation_reused':'Identical non-light XML including physics, home pose and assets'}
   write_json(dest,old)
  write_json(a.output/batch/'manifest.json',manifest)
  previews(a.output/batch,manifest['scenes'])
  print(batch,len(manifest['scenes']),'relit; non-light XML unchanged',flush=True)
if __name__=='__main__':main()
