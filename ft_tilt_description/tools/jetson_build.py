# Jetson visual: NVIDIA's Jetson Orin Nano/NX developer kit (P3768 carrier, P3767 module, fan
# cooler) from its STEP (per-colour STLs from step2stl.py, without the plastic tray and Wi-Fi
# antennas), regrouped by colour, decimated and placed on the F550 top plate on four 8 mm
# standoffs. Visual only. Writes meshes/jetson_*.stl.
# STEP coordinates are mm: carrier PCB 100 x 79 x 1.6 with its bottom face at z = 0, I/O along -y.
import bpy, sys, glob, re, math, mathutils
src, colours, outdir = sys.argv[sys.argv.index("--")+1:][:3]
bpy.ops.wm.read_factory_settings(use_empty=True)

PROTO = {  # group -> STEP colours that map to it (nearest wins); the PCB has no colour (0.5 grey)
    "pcb":    [(0.5, 0.5, 0.5)],
    "metal":  [(0.52, 0.52, 0.52), (0.72, 0.72, 0.72), (0.75, 0.89, 1.0), (0.64, 0.64, 1.0)],
    "white":  [(1.0, 1.0, 1.0), (1.0, 1.0, 0.89)],
    "dark":   [(0.19, 0.19, 0.19), (0.0, 0.0, 0.0), (0.27, 0.29, 0.29), (0.3, 0.28, 0.26)],
    "cooler": [(0.36, 0.43, 0.52)],
    "red":    [(0.79, 0.1, 0.0), (1.0, 0.52, 0.0), (1.0, 0.11, 0.37), (1.0, 0.0, 0.0), (1.0, 0.78, 0.82)],
    "blue":   [(0.03, 0.13, 1.0), (0.49, 0.33, 1.0)],
    "green":  [(0.0, 0.22, 0.0), (0.22, 1.0, 0.52), (0.0, 1.0, 0.0)],
    "gold":   [(0.91, 0.87, 0.58), (1.0, 0.64, 0.14), (0.58, 0.38, 0.04), (1.0, 1.0, 0.0)],
}
def group_of(c):
    return min(((sum((a - b) ** 2 for a, b in zip(c, p)), g) for g, ps in PROTO.items() for p in ps))[1]

cols = {int(m.group(1)): eval(m.group(2)) for m in re.finditer(r"COLOUR (\d+) (\([^)]*\))", open(colours).read())}
groups = {g: [] for g in PROTO}
for f in sorted(glob.glob(f"{src}/part_*.stl")):
    i = int(f[-6:-4]); bpy.ops.wm.stl_import(filepath=f)
    o = bpy.context.selected_objects[0]; o.scale = (1e3, 1e3, 1e3)       # back to mm while building
    bpy.ops.object.transform_apply(scale=True); groups[group_of(cols[i])].append(o)

# ---- four M3 standoffs (8 mm) under the mounting holes (86 x 58 mm, measured from the STEP) ----
HX, HY = (6.0, 92.0), (-3.7, 54.4)
for x in HX:
    for y in HY:
        bpy.ops.mesh.primitive_cylinder_add(vertices=6, radius=2.5, depth=8.0, location=(x, y, -4.0))
        groups["metal"].append(bpy.context.object)

# ---- join, weld, decimate, place (I/O toward the back, holes centred), export ----
TOP_PLATE = -0.030                                    # F550 top plate surface in ft_tilt base_link (m)
BUDGET = 150000                                       # triangles in all
total = sum(len(o.data.polygons) for objs in groups.values() for o in objs)
place = (mathutils.Matrix.Translation((0, 0, TOP_PLATE + 0.008))
         @ mathutils.Matrix.Rotation(math.radians(-90), 4, 'Z')
         @ mathutils.Matrix.Scale(1e-3, 4)
         @ mathutils.Matrix.Translation((-sum(HX) / 2, -sum(HY) / 2, 0)))
for g, objs in groups.items():
    if not objs:
        continue
    bpy.ops.object.select_all(action='DESELECT')
    for o in objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]; bpy.ops.object.join(); j = bpy.context.object
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    bpy.ops.object.mode_set(mode='EDIT'); bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.remove_doubles(threshold=1e-3); bpy.ops.mesh.normals_make_consistent(inside=False)
    bpy.ops.object.mode_set(mode='OBJECT')
    n0 = len(j.data.polygons)
    if total > BUDGET:
        m = j.modifiers.new("d", 'DECIMATE'); m.ratio = BUDGET / total
        bpy.ops.object.modifier_apply(modifier=m.name)
    j.data.transform(place)
    bpy.ops.object.select_all(action='DESELECT'); j.select_set(True)
    bpy.ops.wm.stl_export(filepath=f"{outdir}/jetson_{g}.stl", export_selected_objects=True, ascii_format=False)
    vs = [v.co for v in j.data.vertices]
    print("EXPORT", g, n0, "->", len(j.data.polygons), "tris z", round(min(v[2] for v in vs), 4), round(max(v[2] for v in vs), 4))
