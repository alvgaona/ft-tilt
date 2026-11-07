# 3D-printed 17° tilt adapter (RPIC17 Fig. 4, visual only): flat underside on the F550 motor pad,
# top face square to a spin axis leaning 17° toward the centre (γ = 107°). Arm frame, mm while
# modelling: origin on the rotor plane above the pad centre, x along the arm (outward), z up.
# The motor's mounting face is 26.3 mm below the origin along the axis (a2212.py); the pad top is
# at z = -35 (f550_export.py). Exported in metres. For γ = 73° the xacro turns it by 180°.
import bpy, sys, math, mathutils
outdir = sys.argv[sys.argv.index("--") + 1]
bpy.ops.wm.read_factory_settings(use_empty=True)

G = math.radians(107)
e = mathutils.Vector((math.cos(G), 0, math.sin(G)))       # spin axis (inward lean)
u = mathutils.Vector((math.sin(G), 0, -math.cos(G)))      # in the top face, along the arm
F = -26.3 * e                                             # motor mounting face centre
PAD, HU, HV = -35.0, 18.0, 15.0                           # pad top, half length, half width

top = [F + a * HU * u + mathutils.Vector((0, b * HV, 0)) for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
bot = [mathutils.Vector((p.x, p.y, PAD)) for p in top]
me = bpy.data.meshes.new("adapter")
me.from_pydata(bot + top, [], [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)])
wedge = bpy.data.objects.new("adapter", me)
bpy.context.collection.objects.link(wedge)

# Motor screw holes and shaft clearance, square to the top face (A2212 pattern, a2212.py).
rot = mathutils.Vector((0, 0, 1)).rotation_difference(e).to_euler()
for x, y, r in ((8, 0, 1.6), (-8, 0, 1.6), (0, 9.5, 1.6), (0, -9.5, 1.6), (0, 0, 4.0)):
    c = F + x * u + mathutils.Vector((0, y, 0)) - 3 * e
    bpy.ops.mesh.primitive_cylinder_add(vertices=24, radius=r, depth=8, location=c, rotation=rot)
    t = bpy.context.object
    m = wedge.modifiers.new("b", 'BOOLEAN'); m.operation = 'DIFFERENCE'; m.object = t
    bpy.context.view_layer.objects.active = wedge; bpy.ops.object.modifier_apply(modifier="b")
    bpy.data.objects.remove(t)

m = wedge.modifiers.new("bv", 'BEVEL'); m.width = 0.8; m.segments = 2; m.limit_method = 'ANGLE'
bpy.context.view_layer.objects.active = wedge; bpy.ops.object.modifier_apply(modifier="bv")
wedge.data.transform(mathutils.Matrix.Scale(1e-3, 4))
bpy.ops.object.select_all(action='DESELECT'); wedge.select_set(True)
bpy.ops.wm.stl_export(filepath=f"{outdir}/adapter.stl", export_selected_objects=True, ascii_format=False)
vs = [v.co for v in wedge.data.vertices]
print("EXPORT adapter", len(wedge.data.polygons), "faces, z", round(min(v.z for v in vs), 4), round(max(v.z for v in vs), 4),
      "x", round(min(v.x for v in vs), 4), round(max(v.x for v in vs), 4))
