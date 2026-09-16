"""Run with Blender --background --python; never changes the source blend files."""
import bpy
import json
from pathlib import Path
from mathutils import Vector

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'outputs/r1pro_tray_scene'
report = {}
for key, filename in [('tray', 'Changi_tray_FIXED.blend'), ('table', 'Table_1200x600x800.blend')]:
    bpy.ops.wm.open_mainfile(filepath=str(ROOT.parent / filename), use_scripts=False)
    scene = bpy.context.scene
    objects = [o for o in scene.objects if o.type == 'MESH']
    points = [o.matrix_world @ Vector(v) for o in objects for v in o.bound_box]
    lo = Vector([min(p[i] for p in points) for i in range(3)])
    hi = Vector([max(p[i] for p in points) for i in range(3)])
    offset = Vector(((lo.x + hi.x)/2, (lo.y + hi.y)/2, lo.z))
    bpy.ops.object.select_all(action='DESELECT')
    for o in objects:
        o.location -= offset
        o.select_set(True)
    bpy.context.view_layer.objects.active = objects[0]
    bpy.context.view_layer.update()
    dest = OUT / 'assets' / key
    dest.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.obj_export(filepath=str(dest / f'{key}.obj'), export_selected_objects=True,
        forward_axis='Y', up_axis='Z', apply_modifiers=True, export_triangulated_mesh=True)
    bpy.ops.wm.usd_export(filepath=str(dest / f'{key}.usdc'), selected_objects_only=True,
        export_materials=True, root_prim_path='/Asset')
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.separate(type='MATERIAL')
    bpy.ops.object.mode_set(mode='OBJECT')
    parts=[]
    meshes=[o for o in scene.objects if o.type=='MESH']
    for index,o in enumerate(meshes):
        bpy.ops.object.select_all(action='DESELECT')
        o.select_set(True)
        bpy.context.view_layer.objects.active=o
        part=f'{key}_{index}'
        bpy.ops.wm.obj_export(filepath=str(dest/f'{part}.obj'),export_selected_objects=True,
            forward_axis='Y',up_axis='Z',apply_modifiers=True,export_triangulated_mesh=True)
        mat=o.material_slots[o.data.polygons[0].material_index].material if o.material_slots else None
        rgba=list(mat.diffuse_color) if mat else [0.5,0.5,0.5,1]
        # MuJoCo colors are display RGB; Blender stores linear diffuse colors.
        rgba[:3]=[12.92*v if v<=0.0031308 else 1.055*v**(1/2.4)-0.055 for v in rgba[:3]]
        parts.append({'name':part,'rgba':rgba})
    report[key] = {'dimensions_m': list(hi-lo), 'source_min': list(lo), 'origin': 'bottom center',
        'materials': [(m.name, list(m.diffuse_color)) for m in bpy.data.materials], 'parts':parts}
(OUT / 'object_dimensions.json').write_text(json.dumps(report, indent=2))

