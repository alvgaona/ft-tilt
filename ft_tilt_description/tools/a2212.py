# Hawk's Work A2212 920KV look (visual only), Ø27.7 x 30 mm body. Units mm while modelling,
# z = thrust axis, z = 0 at the mounting face. Exported in metres shifted by -26.3 mm so the
# prop hub (centre 38.3 mm above the mount) lands on the prop plane, 12 mm above the rotor
# link origin in ft_tilt.sdf.xacro.
import bpy, sys, math, mathutils
outdir = sys.argv[sys.argv.index("--") + 1]
bpy.ops.wm.read_factory_settings(use_empty=True)
SEG = 64

def cyl(r, z0, z1, seg=SEG, x=0.0, y=0.0):
    bpy.ops.mesh.primitive_cylinder_add(vertices=seg, radius=r, depth=z1 - z0, location=(x, y, (z0 + z1) / 2))
    return bpy.context.object

def cone(r0, r1, z0, z1, seg=SEG):
    bpy.ops.mesh.primitive_cone_add(vertices=seg, radius1=r0, radius2=r1, depth=z1 - z0, location=(0, 0, (z0 + z1) / 2))
    return bpy.context.object

def box(sx, sy, sz, x, y, z, rz=0.0):
    bpy.ops.mesh.primitive_cube_add(size=1, location=(x, y, z), rotation=(0, 0, rz))
    o = bpy.context.object; o.scale = (sx, sy, sz); bpy.ops.object.transform_apply(scale=True)
    return o

def apply(o, m):
    bpy.context.view_layer.objects.active = o; bpy.ops.object.modifier_apply(modifier=m.name)

def boolean(target, op, *tools):
    for t in tools:
        m = target.modifiers.new("b", 'BOOLEAN'); m.operation = op; m.object = t
        apply(target, m); bpy.data.objects.remove(t)
    return target

def bevel(o, w, seg=2):
    m = o.modifiers.new("bv", 'BEVEL'); m.width = w; m.segments = seg; m.limit_method = 'ANGLE'
    apply(o, m); return o

def join(objs):
    bpy.ops.object.select_all(action='DESELECT')
    for o in objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]; bpy.ops.object.join()
    return bpy.context.object

R, RI = 27.7 / 2, 12.6
# ---------------- static: red base, copper stator ----------------
base = bevel(cyl(R, 0, 4.5), 0.6)
boolean(base, 'DIFFERENCE', cyl(3.2, -1, 6, 32),
        *[cyl(1.6, -1, 2.5, 24, x, y) for x, y in ((8, 0), (-8, 0), (0, 9.5), (0, -9.5))])
core = cyl(8.0, 4.5, 25.5, 36)
coils = []
for k in range(12):
    a = math.radians(30 * k)
    c = box(4.6, 4.0, 19.0, 9.6 * math.cos(a), 9.6 * math.sin(a), 16.0, a)
    coils.append(bevel(c, 1.2, 3))                      # rounded copper bundles
stator = join([core, *coils])

# ---------------- rotating: dark sleeve, red cap + shaft, black spinner ----------------
band = boolean(cyl(R, 5.0, 19.0), 'DIFFERENCE', cyl(RI, 4, 20))
cap = bevel(cyl(R, 19.0, 29.5), 1.0, 3)
boolean(cap, 'DIFFERENCE', cyl(RI, 18, 27.5))           # cup, 2 mm top
spokes_at = [90, 210, 330]
boolean(cap, 'DIFFERENCE', *[cyl(8.0, 22.0, 31.0, 48, 12.5 * math.cos(math.radians(a - 60)),
                                     12.5 * math.sin(math.radians(a - 60))) for a in spokes_at])
hub = cyl(5.5, 26.0, 30.5, 48)
collar = bevel(cyl(5.2, 30.5, 33.5, 6), 0.4)            # hex on the red shaft
thread = cyl(3.0, 33.5, 43.0, 32)
ridges = [cyl(3.35, z, z + 0.45, 32) for z in [34.0 + 1.0 * k for k in range(9)]]
red_top = join([cap, hub, collar, thread, *ridges])

nut = bevel(cyl(5.6, 43.0, 45.5, 6), 0.4)               # black bullet nut
bullet = cone(5.0, 1.8, 45.5, 51.0)
bpy.ops.mesh.primitive_uv_sphere_add(radius=1.8, location=(0, 0, 51.0), segments=24, ring_count=12)
spinner = join([nut, bullet, bpy.context.object])

for o, name in ((base, "motor_base"), (stator, "motor_stator"), (band, "motor_band"),
                (red_top, "motor_bell"), (spinner, "motor_spinner")):
    bpy.ops.object.select_all(action='DESELECT'); o.select_set(True)
    bpy.context.view_layer.objects.active = o
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    o.data.transform(mathutils.Matrix.Scale(1e-3, 4) @ mathutils.Matrix.Translation((0, 0, -26.3)))
    bpy.ops.object.mode_set(mode='EDIT'); bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.normals_make_consistent(inside=False); bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.wm.stl_export(filepath=f"{outdir}/{name}.stl", export_selected_objects=True, ascii_format=False)
    vs = [v.co for v in o.data.vertices]
    print("EXPORT", name, len(o.data.polygons), "tris z", round(min(v[2] for v in vs), 4), round(max(v[2] for v in vs), 4))
