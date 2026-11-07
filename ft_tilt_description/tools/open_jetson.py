# Blender GUI viewer: the Jetson visual meshes in collections, coloured as in the xacro, in mm.
# blender -P open_jetson.py [-- out.blend]
import bpy, os, sys
M = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "meshes")
OUT = sys.argv[sys.argv.index("--") + 1] if "--" in sys.argv else None
TREE = {  # collection -> [(stl group, object name, colour)]
    "NVIDIA dev kit (P3768 + P3767)": [
        ("pcb", "PCBs and fan", (0.07, 0.08, 0.07)), ("metal", "Shields, shells and standoffs", (0.7, 0.7, 0.72)),
        ("white", "White parts", (0.9, 0.9, 0.88)), ("dark", "ICs and plastics", (0.08,) * 3),
        ("cooler", "Cooler", (0.36, 0.43, 0.52)), ("red", "USB and screws", (0.75, 0.2, 0.05)),
        ("blue", "Blue parts", (0.1, 0.25, 0.85)), ("green", "SO-DIMM and M.2 boards", (0.2, 0.6, 0.35)),
        ("gold", "Gold parts", (0.85, 0.65, 0.2))],
}
for o in list(bpy.data.objects):
    bpy.data.objects.remove(o)
for c in list(bpy.data.collections):
    bpy.data.collections.remove(c)
root = bpy.data.collections.new("Jetson dev kit"); bpy.context.scene.collection.children.link(root)
for cname, items in TREE.items():
    col = bpy.data.collections.new(cname); root.children.link(col)
    for g, name, c in items:
        bpy.ops.wm.stl_import(filepath=f"{M}/jetson_{g}.stl")
        o = bpy.context.selected_objects[0]; o.name = o.data.name = name; o.scale = (1e3,) * 3
        for uc in list(o.users_collection):
            uc.objects.unlink(o)
        col.objects.link(o)
        m = bpy.data.materials.new(name); m.diffuse_color = (*c, 1)
        m.use_nodes = True; m.node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value = (*c, 1)
        o.data.materials.append(m)
if OUT:
    bpy.ops.wm.save_as_mainfile(filepath=OUT)
def view():
    for a in bpy.context.screen.areas:
        if a.type == 'VIEW_3D':
            a.spaces[0].shading.type = 'MATERIAL'; a.spaces[0].clip_start = 0.1; a.spaces[0].clip_end = 10000
            with bpy.context.temp_override(area=a, region=[r for r in a.regions if r.type == 'WINDOW'][0]):
                bpy.ops.object.select_all(action='SELECT'); bpy.ops.view3d.view_all(); bpy.ops.object.select_all(action='DESELECT')
bpy.app.timers.register(view, first_interval=1.0)
