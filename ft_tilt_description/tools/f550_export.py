# Splits the F550 STL (mm, Z up) into body / arm-1 / other-arms meshes in metres, aligned to
# ft_tilt: arms on motors 1..6 at (i-1)·60°, motor pads at r = 0.26 m, pads just below z = 0.
import bpy, sys, math, mathutils
src, outdir = sys.argv[sys.argv.index("--")+1:][:2]
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.wm.stl_import(filepath=src)
ob = bpy.context.selected_objects[0]; bpy.context.view_layer.objects.active = ob
bpy.ops.object.mode_set(mode='EDIT'); bpy.ops.mesh.select_all(action='SELECT')
bpy.ops.mesh.remove_doubles(threshold=0.01)
bpy.ops.mesh.separate(type='LOOSE'); bpy.ops.object.mode_set(mode='OBJECT')

def centre(o):
    vs = [o.matrix_world @ v.co for v in o.data.vertices]
    return [(min(v[i] for v in vs) + max(v[i] for v in vs)) / 2 for i in range(3)]

objs = [o for o in bpy.context.scene.objects if o.type == 'MESH']
motors = [o for o in objs if math.hypot(*centre(o)[:2]) > 240 and len(o.data.polygons) == 412]
mc = [centre(o) for o in motors]
cx = sum(c[0] for c in mc) / len(mc); cy = sum(c[1] for c in mc) / len(mc)
r_motor = sum(math.hypot(c[0] - cx, c[1] - cy) for c in mc) / len(mc)
pad_top = min(c[2] for c in mc) - 16.5          # motor bottom = top of the motor pad
s = 0.26 / (r_motor * 1e-3) * 1e-3              # mm → m, motor radius → arm 0.26 m
print("motor centre", cx, cy, "radius mm", r_motor, "pad top mm", pad_top, "scale", s)

groups = {"f550_body": [], "f550_arm1": [], "f550_arms": []}
for o in objs:
    c = centre(o); r = math.hypot(c[0] - cx, c[1] - cy)
    ang = math.degrees(math.atan2(c[1] - cy, c[0] - cx))
    if r > 240:
        bpy.data.objects.remove(o)              # motors, props, shafts, pad screws: ours replace them
    elif c[2] > 60 and r < 150:
        bpy.data.objects.remove(o)              # GPS mast and puck
    elif 150 < r < 240 and c[2] > 0:
        groups["f550_arm1" if abs(ang - 30) < 15 else "f550_arms"].append(o)
    else:
        groups["f550_body"].append(o)

xf = (mathutils.Matrix.Translation((0, 0, -0.035))           # pads 3.5 cm under the rotor plane
      @ mathutils.Matrix.Rotation(math.radians(-30), 4, 'Z')
      @ mathutils.Matrix.Scale(s, 4)
      @ mathutils.Matrix.Translation((-cx, -cy, -pad_top)))
for name, members in groups.items():
    bpy.ops.object.select_all(action='DESELECT')
    for o in members:
        o.select_set(True)
    bpy.context.view_layer.objects.active = members[0]
    bpy.ops.object.join()
    j = bpy.context.view_layer.objects.active
    j.data.transform(xf); j.matrix_world = mathutils.Matrix.Identity(4)
    # SketchUp export has inward faces; Gazebo culls back faces, so point them all outward.
    bpy.ops.object.mode_set(mode='EDIT'); bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.normals_make_consistent(inside=False); bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.select_all(action='DESELECT'); j.select_set(True)
    bpy.ops.wm.stl_export(filepath=f"{outdir}/{name}.stl", export_selected_objects=True, ascii_format=False)
    vs = [v.co for v in j.data.vertices]
    print("EXPORT", name, len(members), "parts", len(j.data.polygons), "tris",
          "min", [round(min(v[i] for v in vs), 3) for i in range(3)],
          "max", [round(max(v[i] for v in vs), 3) for i in range(3)])
