# Blender GUI viewer: the Jetson visual meshes in collections, coloured as in the xacro, in mm.
# blender -P open_jetson.py [-- out.blend]
import bpy, os, sys
M = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "meshes")
OUT = sys.argv[sys.argv.index("--") + 1] if "--" in sys.argv else None
TREE = {  # collection -> [(stl group, object name, colour)]
    "Carrier (Auvidea JNX42)": [
        ("pcb", "Carrier PCB", (0.03, 0.08, 0.3)), ("silver", "Carrier connectors and shields", (0.6,) * 3),
        ("dark", "Carrier ICs and plastics", (0.06,) * 3), ("blue", "Carrier blue parts", (0.1, 0.3, 0.9)),
        ("copper", "Carrier copper parts", (0.55, 0.3, 0.12)), ("gold", "Carrier gold parts", (1, 0.75, 0.1)),
        ("green", "Carrier green parts", (0.3, 0.7, 0.1))],
    "Orin NX module": [
        ("mod_pcb", "Module PCB", (0.02, 0.02, 0.025)), ("mod_gold", "Module edge fingers and SoC caps", (1, 0.75, 0.1)),
        ("mod_copper", "Module pads and passives", (0.55, 0.3, 0.12)), ("mod_substrate", "SoC substrate", (0.1, 0.35, 0.38)),
        ("mod_lid", "SoC lid and terminals", (0.8, 0.8, 0.82)), ("mod_chip", "DRAM, power ICs and inductor print", (0.07, 0.07, 0.08)),
        ("mod_ind", "Inductor bodies", (0.45, 0.45, 0.47)), ("mod_mark", "Markings", (0.42, 0.42, 0.44))],
    "Cooler": [
        ("shroud", "Shroud, fins and emboss", (0.05,) * 3), ("fan", "Fan, cable and connector", (0.02,) * 3)],
}
for o in list(bpy.data.objects):
    bpy.data.objects.remove(o)
for c in list(bpy.data.collections):
    bpy.data.collections.remove(c)
root = bpy.data.collections.new("Jetson Orin NX on JNX42"); bpy.context.scene.collection.children.link(root)
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
