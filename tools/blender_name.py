"""
Blender version of tools/make_name_mesh.py - edit the name on the shoulder link in Blender.

    blender --background --python tools/blender_name.py -- --text "YOUR NAME"
    (or open Blender > Scripting > Open this file > Run Script, after editing TEXT below)

It imports meshes/shoulder_link.stl, adds a Text object on the flat top face of the
arm (centred, reading from the column towards the elbow), extrudes it, converts it to
a mesh and exports ONLY the letters to meshes/shoulder_name.stl - in the same
millimetre frame as shoulder_link.stl, so the URDF / MuJoCo / app need no change.
The shoulder mesh stays in the scene so you can move / restyle the letters by hand
and export again (select the name object, File > Export > STL, "Selection only", scale 1).

Works with Blender 3.x and 4.x. The pure-Python tool (make_name_mesh.py) gives the
exact same placement and is what produced the shipped shoulder_name.stl.
"""
import os
import sys

import bpy

TEXT = 'YOUR NAME'                 # or pass --text
CAP_MM = 19.0           # letter height
DEPTH_MM = 1.6          # how far the letters stand out of the face
SINK_MM = 0.5           # how far they go into the face (no gap)
FONT = ''               # optional .ttf/.otf path, '' = Blender's default font

# frame of shoulder_link.stl (mm): top face and the visual centre of the arm
MESH_OFFSET_MM = (-59.954333, -59.974857, -390.0)
TOP_FACE_Z_MM = 27.0 - MESH_OFFSET_MM[2]          # 417.0
CENTER_X_MM = 160.0 - MESH_OFFSET_MM[0]          # 219.95
CENTER_Y_MM = 0.0 - MESH_OFFSET_MM[1]            # 59.97

argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
if '--text' in argv:
    TEXT = argv[argv.index('--text') + 1]

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MESHES = os.path.join(ROOT, 'meshes')


def import_stl(path):
    if hasattr(bpy.ops.wm, 'stl_import'):           # Blender 4.x
        bpy.ops.wm.stl_import(filepath=path)
    else:                                           # Blender 3.x
        bpy.ops.import_mesh.stl(filepath=path)
    return bpy.context.selected_objects[0]


def export_stl(obj, path):
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    if hasattr(bpy.ops.wm, 'stl_export'):           # Blender 4.x
        bpy.ops.wm.stl_export(filepath=path, export_selected_objects=True, global_scale=1.0, ascii_format=False)
    else:
        bpy.ops.export_mesh.stl(filepath=path, use_selection=True, global_scale=1.0, ascii=False)


bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete()
bpy.context.scene.unit_settings.scale_length = 0.001        # 1 Blender unit = 1 mm
shoulder = import_stl(os.path.join(MESHES, 'shoulder_link.stl'))
shoulder.name = 'shoulder_link'

bpy.ops.object.text_add()
txt = bpy.context.active_object
txt.name = 'shoulder_name'
txt.data.body = TEXT
if FONT:
    txt.data.font = bpy.data.fonts.load(FONT)
txt.data.align_x = 'CENTER'
txt.data.align_y = 'CENTER'
txt.data.size = CAP_MM / 0.70            # Blender size = em height; capitals are ~70 % of it
txt.data.extrude = (DEPTH_MM + SINK_MM) / 2
txt.location = (CENTER_X_MM, CENTER_Y_MM, TOP_FACE_Z_MM + (DEPTH_MM - SINK_MM) / 2)
bpy.ops.object.convert(target='MESH')
name = bpy.context.active_object

# exact centring on the face (align_y='CENTER' is only approximate for capitals)
bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY', center='BOUNDS')
name.location.x, name.location.y = CENTER_X_MM, CENTER_Y_MM
bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)

mat = bpy.data.materials.new('name_white')
mat.diffuse_color = (0.96, 0.96, 0.96, 1)
name.data.materials.append(mat)

out = os.path.join(MESHES, 'shoulder_name.stl')
export_stl(name, out)
print('exported', out)
