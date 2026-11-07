# Jetson visual: Auvidea JNX42 carrier from its STEP (per-colour STLs from step2stl.py, regrouped
# and decimated), plus an Orin NX module and NVIDIA-style cooler modelled here, placed on the F550
# top plate on four 8 mm standoffs. Visual only. Writes meshes/jetson_*.stl.
# Board coordinates are mm: carrier PCB 80 x 104.6 x 1.5 with its top face at z = 0.
# Text uses macOS system fonts (/System/Library/Fonts/Supplemental).
import bpy, sys, glob, re, math, mathutils
src, colours, outdir = sys.argv[sys.argv.index("--")+1:][:3]
bpy.ops.wm.read_factory_settings(use_empty=True)

PROTO = {  # group -> prototype colours that map to it (nearest wins)
    "pcb":    [(0.01, 0.04, 0.15)],
    "silver": [(0.55, 0.55, 0.55), (1.0, 1.0, 1.0)],
    "dark":   [(0.1, 0.1, 0.1)],
    "blue":   [(0.1, 0.3, 0.9)],
    "copper": [(0.5, 0.25, 0.05)],
    "gold":   [(1.0, 0.75, 0.1)],
    "green":  [(0.0, 0.7, 0.1)],
}
def group_of(c):
    return min(((sum((a - b) ** 2 for a, b in zip(c, p)), g) for g, ps in PROTO.items() for p in ps))[1]

cols = {int(m.group(1)): eval(m.group(2)) for m in re.finditer(r"COLOUR (\d+) (\([^)]*\))", open(colours).read())}
groups = {g: [] for g in PROTO}
for f in sorted(glob.glob(f"{src}/part_*.stl")):
    i = int(f[-6:-4]); bpy.ops.wm.stl_import(filepath=f)
    o = bpy.context.selected_objects[0]; o.scale = (1e3, 1e3, 1e3)       # back to mm while building
    bpy.ops.object.transform_apply(scale=True); groups[group_of(cols[i])].append(o)

def box(sx, sy, sz, x, y, z):
    bpy.ops.mesh.primitive_cube_add(size=1, location=(x, y, z))
    o = bpy.context.object; o.scale = (sx, sy, sz); bpy.ops.object.transform_apply(scale=True); return o
def cyl(r, z0, z1, x, y, seg=48):
    bpy.ops.mesh.primitive_cylinder_add(vertices=seg, radius=r, depth=z1 - z0, location=(x, y, (z0 + z1) / 2))
    return bpy.context.object
def cut(t, *tools):
    for tool in tools:
        m = t.modifiers.new("b", 'BOOLEAN'); m.operation = 'DIFFERENCE'; m.object = tool
        bpy.context.view_layer.objects.active = t; bpy.ops.object.modifier_apply(modifier=m.name)
        bpy.data.objects.remove(tool)
    return t

# ---- Orin NX module + thermal solution, in the SO-DIMM slot (x 30-36, y 20-96, top 9.2) ----
MX0, MX1, MY = 31.0, 76.0, 58.05          # module spans x 31..76, centred on the slot in y
W, L = 69.6, MX1 - MX0
z_pcb = 5.2
# Cooler after NVIDIA's Orin NX/Nano reference thermal solution: 58 x 38 x 18 mm, centred on the
# module. Enclosed shroud, fins along the long axis open at both ends, a centred 11-blade fan,
# and its cable down to a connector on the module. Details (fan, fins) from photos, sizes assumed.
groups["shroud"], groups["fan"] = [], []
cx = (MX0 + MX1) / 2
CL, CW = 58.0, 38.0                              # cooler length (along the module's 69.6) and width
SY0, SY1 = MY - CL / 2, MY + CL / 2
H, T = 18.0, 1.2                                 # shroud height, wall
z0 = z_pcb + 1.0 + 2.0                           # SoC + thermal pad
top = z0 + H
R_HOLE = 17.0
fy = (SY0 + SY1) / 2                             # fan centred
def bevel(o, w, seg=4):
    m = o.modifiers.new("bv", 'BEVEL'); m.width = w; m.segments = seg; m.limit_method = 'ANGLE'
    bpy.context.view_layer.objects.active = o; bpy.ops.object.modifier_apply(modifier=m.name); return o
outer = bevel(box(CW, SY1 - SY0, H, cx, (SY0 + SY1) / 2, z0 + H / 2), 0.6, 2)   # near-sharp edges
inner = box(CW - 2 * T, SY1 - SY0 + 4.0, H - T + 1.0, cx, fy, z0 + (H - T - 1.0) / 2)   # open at both ends
shroud = cut(outer, inner, cyl(R_HOLE, top - 3, top + 1, cx, fy, 128))
groups["shroud"].append(shroud)
groups["shroud"].append(box(CW - 2 * T, SY1 - SY0, 2.5, cx, fy, z0 + 1.25))   # base
NF, FT = 6, 1.8                                  # 6 fins -> 7 slots in each end face
fins = []
for k in range(NF):
    slot = (CW - 2 * T - NF * FT) / (NF + 1)
    x = cx - CW / 2 + T + slot * (k + 1) + FT * k + FT / 2
    fins.append(box(FT, SY1 - SY0, H - T - 2.5, x, fy, z0 + 2.5 + (H - T - 2.5) / 2))
bpy.ops.object.select_all(action='DESELECT')
for f in fins:
    f.select_set(True)
bpy.context.view_layer.objects.active = fins[0]; bpy.ops.object.join()
groups["shroud"].append(cut(bpy.context.object, cyl(R_HOLE + 0.5, top - T - 11.0, top + 1, cx, fy, 128)))  # fan pocket

# Fan: big flat hub with a centre dimple, 11 swept, pitched, cambered blades.
hub = cut(bevel(cyl(10.0, top - 10.5, top - 0.6, cx, fy, 96), 0.6, 3), cyl(1.0, top - 1.0, top, cx, fy, 24))
groups["fan"].append(hub)
NB, R0, R1, NR, NS = 11, 9.5, 16.6, 10, 12
for k in range(NB):
    verts, faces = [], []
    for i in range(NR):
        t = i / (NR - 1); r = R0 + t * (R1 - R0)
        th = 2 * math.pi * k / NB + 0.45 * t      # swept forward toward the tip
        dphi = 0.40 + 0.12 * t                    # wider at the tip
        h = 6.5 - 1.5 * t                         # axial depth: steeper at the root
        for j in range(NS):
            u = j / (NS - 1) - 0.5
            ph = th + u * dphi
            zz = top - 5.5 + u * h - 0.5 * (1 - (2 * u) ** 2)   # pitch + camber
            verts.append((cx + r * math.cos(ph), fy + r * math.sin(ph), zz))
    for i in range(NR - 1):
        for j in range(NS - 1):
            q = i * NS + j; faces.append((q, q + 1, q + NS + 1, q + NS))
    me = bpy.data.meshes.new(f"blade{k}"); me.from_pydata(verts, [], faces)
    o = bpy.data.objects.new(f"blade{k}", me); bpy.context.scene.collection.objects.link(o)
    m = o.modifiers.new("s", 'SOLIDIFY'); m.thickness = 0.7; m.offset = 0
    bpy.context.view_layer.objects.active = o; o.select_set(True); bpy.ops.object.modifier_apply(modifier=m.name)
    groups["fan"].append(o)

# Cable: leaves the end face near the top, bends down to a 4-pin connector on the bare module edge.
cxk = MX1 - 6.0
zc = z_pcb + 1.0 + 4.0                            # connector top
pts = [(cxk, SY1 - 1.0, top - 3.0)]
for i in range(13):
    a2 = math.pi / 2 * i / 12
    pts.append((cxk, SY1 + 1.5 * math.sin(a2), top - 4.5 + 1.5 * math.cos(a2)))
pts.append((cxk, SY1 + 1.5, zc))
cu = bpy.data.curves.new("cable", 'CURVE'); cu.dimensions = '3D'; cu.bevel_depth = 0.9; cu.bevel_resolution = 3
sp = cu.splines.new('POLY'); sp.points.add(len(pts) - 1)
for pnt, c in zip(sp.points, pts):
    pnt.co = (*c, 1)
co = bpy.data.objects.new("cable", cu); bpy.context.scene.collection.objects.link(co)
bpy.ops.object.select_all(action='DESELECT'); co.select_set(True); bpy.context.view_layer.objects.active = co
bpy.ops.object.convert(target='MESH'); groups["fan"].append(bpy.context.object)
groups["fan"].append(box(6.0, 2.8, 4.0, cxk, SY1 + 1.5, z_pcb + 1.0 + 2.0))     # connector
# ---- Orin NX module (69.6 x 45 mm). Layout measured from a top-down product photo (~18.9 px/mm);
# heights assumed. Module coords: u from the finger edge (x = MX0) toward the far edge, v along the
# 69.6 mm side (+v is right in the photo, fingers at the top). Markings read along +v, top toward -u.
import random
for g in ("mod_pcb", "mod_substrate", "mod_lid", "mod_chip", "mod_ind", "mod_gold", "mod_copper", "mod_mark"):
    groups[g] = []
zt = z_pcb + 1.0                                  # module top surface
rng = random.Random(1)
def X(u): return MX0 + u
def Y(v): return MY + v
def boxes(name, specs):
    """One mesh from many boxes: (sx, sy, sz, x, y, z_centre) in board coords."""
    vs, fs = [], []
    for sx, sy, sz, x, y, z in specs:
        b = len(vs)
        vs += [(x + i * sx / 2, y + j * sy / 2, z + k * sz / 2) for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]
        fs += [(b, b + 1, b + 3, b + 2), (b + 4, b + 6, b + 7, b + 5), (b, b + 4, b + 5, b + 1),
               (b + 2, b + 3, b + 7, b + 6), (b, b + 2, b + 6, b + 4), (b + 1, b + 5, b + 7, b + 3)]
    me = bpy.data.meshes.new(name); me.from_pydata(vs, [], fs)
    o = bpy.data.objects.new(name, me); bpy.context.scene.collection.objects.link(o); return o
FONTS = {k: bpy.data.fonts.load(f) for k, f in (
    ("bold", "/System/Library/Fonts/Supplemental/Arial Bold.ttf"),
    ("black", "/System/Library/Fonts/Supplemental/Arial Black.ttf"),
    ("narrow", "/System/Library/Fonts/Supplemental/Arial Narrow.ttf"),
    ("italic", "/System/Library/Fonts/Supplemental/Arial Bold Italic.ttf"),
    ("mono", "/System/Library/Fonts/Supplemental/Andale Mono.ttf"))}
def text(body, size, x, y, z, rot_z, font="bold", h=0.03, offset=0.0):
    cu = bpy.data.curves.new("t", 'FONT'); cu.body = body; cu.size = size; cu.font = FONTS[font]
    cu.align_x = 'CENTER'; cu.align_y = 'CENTER'; cu.extrude = h / 2; cu.resolution_u = 3; cu.offset = offset
    o = bpy.data.objects.new("t", cu); bpy.context.scene.collection.objects.link(o)
    o.location = (x, y, z + h / 2); o.rotation_euler = (0, 0, math.radians(rot_z))
    bpy.ops.object.select_all(action='DESELECT'); o.select_set(True); bpy.context.view_layer.objects.active = o
    bpy.ops.object.convert(target='MESH'); bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    return bpy.context.object
def outline_text(body, size, x, y, z, rot_z, font, w):
    return cut(text(body, size, x, y, z, rot_z, font), text(body, size, x, y, z - 0.05, rot_z, font, 0.2, -w))
def ring(r0, r1, x, y, z, h=0.03, seg=40):
    return cut(cyl(r1, z, z + h, x, y, seg), cyl(r0, z - 1, z + 1, x, y, seg))

# PCB: key notch 3.1 mm left of centre, steps beside the fingers, 8 holes.
HOLES = [(10.0, -21.7, 1.6), (10.0, 21.7, 1.6), (39.1, -21.7, 1.6), (39.1, 21.7, 1.6),   # big, plated
         (41.8, -32.0, 1.3), (41.8, 32.0, 1.3), (5.4, -33.7, 0.8), (5.4, 33.7, 0.8)]
pcb = box(L, W, 1.0, X(L / 2), MY, z_pcb + 0.5)
tools = [box(5.0, 1.0, 3.0, X(1.5), Y(-3.1), zt - 0.5), cyl(0.5, z_pcb - 1, zt + 1, X(4.0), Y(-3.1), 16)]
tools += [box(5.0, 2.4, 3.0, X(0.0), Y(sv * W / 2), zt - 0.5) for sv in (-1, 1)]
tools += [cyl(r, z_pcb - 1, zt + 1, X(u), Y(v), 32) for u, v, r in HOLES]
groups["mod_pcb"].append(cut(pcb, *tools))
for u, v, r in HOLES[:6]:
    for zz in (zt, z_pcb - 0.03):
        groups["mod_copper"].append(ring(r, r + 0.5, X(u), Y(v), zz))
for u, v, r in HOLES[4:6]:                                                     # via ring round the corner holes
    for k in range(8):
        a = 2 * math.pi * k / 8
        groups["mod_copper"].append(cyl(0.22, zt, zt + 0.03, X(u + 2.2 * math.cos(a)), Y(v + 2.2 * math.sin(a)), 12))
fing, v = [], -32.6                                                            # 0.5 mm pitch, both faces
while v <= 32.6:
    if abs(v + 3.1) > 0.8:
        fing += [(2.4, 0.3, 0.04, X(1.25), Y(v), zz) for zz in (zt + 0.02, z_pcb - 0.02)]
    v += 0.5
groups["mod_gold"].append(boxes("fingers", fing))

# SoC: 38 mm substrate, 21.4 x 23.5 mm lid, one row of capacitors round the lid, pin-1 triangle,
# fiducials and a data-matrix code.
SU, SV, A, B = 24.8, -0.2, 10.7, 11.75                                         # lid centre and half-sizes
groups["mod_substrate"].append(box(37.9, 38.0, 1.0, X(24.75), Y(-0.1), zt + 0.5))
groups["mod_lid"].append(bevel(box(2 * A, 2 * B, 1.0, X(SU), Y(SV), zt + 1.5), 0.4, 3))
caps, zs = [], zt + 1.11
for u_row, v0, v1 in ((12.0, -8.8, 10.5), (37.1, -12.8, 12.3)):
    v = v0
    while v <= v1:
        if rng.random() > 0.12:
            caps.append((0.6, 0.3, 0.22, X(u_row), Y(v), zs))
        v += 0.63
for v_col, u0, u1 in ((-13.3, 15.5, 33.9), (13.1, 19.2, 33.9)):
    u = u0
    while u <= u1:
        if rng.random() > 0.12:
            caps.append((0.3, 0.6, 0.22, X(u), Y(v_col), zs))
        u += 0.63
groups["mod_gold"].append(boxes("soc_caps", caps))
tri = bpy.data.meshes.new("pin1"); z1 = zt + 1.0
tri.from_pydata([(X(5.9), Y(-18.9), z1), (X(5.9), Y(-18.0), z1), (X(6.8), Y(-18.9), z1),
                 (X(5.9), Y(-18.9), z1 + 0.03), (X(5.9), Y(-18.0), z1 + 0.03), (X(6.8), Y(-18.9), z1 + 0.03)],
                [], [(0, 2, 1), (3, 4, 5), (0, 1, 4, 3), (1, 2, 5, 4), (2, 0, 3, 5)])
to = bpy.data.objects.new("pin1", tri); bpy.context.scene.collection.objects.link(to); groups["mod_gold"].append(to)
dm = [(0.14, 0.14, 0.03, X(10.4 + 0.14 * i), Y(12.9 + 0.14 * j), z1 + 0.015)
      for i in range(10) for j in range(10) if i == 0 or j == 0 or rng.random() < 0.5]
dm += [(0.4, 0.4, 0.03, X(u), Y(v), z1 + 0.015) for u, v in ((14.0, 13.6), (37.0, -14.0))]   # fiducials
groups["mod_mark"].append(boxes("datamatrix", dm))
lid_top = zt + 2.0
groups["mod_mark"] += [outline_text("NVIDIA", 2.9, X(19.4), Y(-1.2), lid_top, 90, "black", 0.12),
                       ring(0.36, 0.46, X(20.0), Y(6.8), lid_top, seg=24),
                       text("R", 0.5, X(20.0), Y(6.8), lid_top, 90, "bold"),
                       text("S  TW  2207A1", 1.8, X(24.0), Y(-0.3), lid_top, 90, "mono"),
                       text("SNT69K.M03", 1.8, X(26.1), Y(-2.6), lid_top, 90, "mono"),
                       ring(0.68, 0.82, X(26.1), Y(6.5), lid_top, seg=32),
                       text("e2", 0.9, X(26.1), Y(6.5), lid_top, 90, "bold"),
                       text("TE980M-A1", 1.8, X(28.9), Y(0.2), lid_top, 90, "mono")]

# Power ICs (QFN) by the fingers, grey marking, upside down as in the photo.
for sv in (-1, 1):
    u, v = 9.9, sv * 26.9
    groups["mod_chip"].append(bevel(box(6.4, 5.6, 0.9, X(u), Y(v), zt + 0.45), 0.1, 1))
    groups["mod_mark"] += [text("6A1P2Y", 0.9, X(u - 0.7), Y(v), zt + 0.9, 270, "mono"),
                           text("0E693", 0.9, X(u + 0.7), Y(v), zt + 0.9, 270, "mono")]

# DRAM (Micron LPDDR5): 14.4 x 14.3 mm, the left one rotated 180 deg. Simplified Micron mark:
# a tilted elliptical swoosh ring round an italic m.
def micron(x, y, z, rot):
    e = cut(cyl(1.0, z, z + 0.03, x, y, 40), cyl(0.8, z - 1, z + 1, x, y, 40))
    e.data.transform(mathutils.Matrix.Rotation(math.radians(rot + 20), 4, 'Z')        # mesh data is local to (x, y)
                     @ mathutils.Matrix.Diagonal((1.0, 1.7, 1.0, 1.0)))
    return [e, text("m", 1.6, x, y, z, rot, "italic")]
for sv, rot in ((1, 90), (-1, 270)):
    u0, v0 = 29.9, sv * 26.55
    d = lambda du, dv: (X(u0 + sv * du), Y(v0 + sv * dv))                       # offsets turn with the chip
    groups["mod_chip"].append(bevel(box(14.4, 14.3, 1.0, X(u0), Y(v0), zt + 0.5), 0.2, 1))
    zd = zt + 1.0
    groups["mod_mark"] += [text("20B77", 1.8, *d(-2.3, -1.55), zd, rot, "narrow"),
                           text("D8BFN", 1.8, *d(-0.3, -1.55), zd, rot, "narrow"),
                           text("7ERV" if sv > 0 else "7DGJ", 0.8, *d(-4.4, -6.2), zd, rot - 90, "narrow"),
                           cyl(0.35, zd, zd + 0.03, *d(5.9, -6.05), 16)]                      # pin-1 dot
    groups["mod_mark"] += micron(*d(-1.2, 3.15), zd, rot)

# Inductors: grey bodies with silver end terminals. R20 has dot-matrix print; 1R0 and 4R7 solid.
DOTS = {"R": ["11110", "10001", "10001", "11110", "10100", "10010", "10001"],
        "2": ["01110", "10001", "00001", "00010", "00100", "01000", "11111"],
        "0": ["01110", "10001", "10001", "10001", "10001", "10001", "01110"]}
def dot_text(s, uc, vc, z, p):
    """Dot-matrix text, upside down (reads along -v, top toward +u) as on the module."""
    out = []
    for k, ch in enumerate(s):
        for r, row in enumerate(DOTS[ch]):
            for c, bit in enumerate(row):
                if bit == "1":
                    j = k * 6 + c
                    out.append(cyl(0.2, z, z + 0.03, X(uc + (3 - r) * p), Y(vc - (j - 8) * p), 10))
    return out
for sv in (-1, 1):
    u, v, h = 18.2, sv * 25.9, 1.9
    groups["mod_ind"].append(bevel(box(7.1, 9.5, h, X(u), Y(v), zt + h / 2), 0.3, 2))
    groups["mod_lid"] += [box(4.0, 0.6, 1.2, X(u), Y(v + s * 4.9), zt + 0.6) for s in (-1, 1)]
    groups["mod_chip"] += dot_text("R20", u, v, zt + h, 0.55)
for sv, label in ((-1, "1R0"), (1, "4R7")):
    u, v, h = 39.4, sv * 26.5, 1.6
    groups["mod_ind"].append(bevel(box(3.8, 4.8, h, X(u), Y(v), zt + h / 2), 0.25, 2))
    groups["mod_lid"] += [box(2.6, 0.4, 0.9, X(u), Y(v + s * 2.45), zt + 0.45) for s in (-1, 1)]
    groups["mod_chip"].append(text(label, 2.4, X(u), Y(v), zt + h, 270, "bold"))

# 0603 passives where the photo shows them: tan body, silver ends. (u, v, long axis)
PAS = []
for sv in (-1, 1):
    PAS += [(6.8, sv * dv, "u") for dv in (23.3, 22.2, 21.0)]
    PAS += [(uu, sv * 31.2, "v") for uu in (5.6, 7.1, 8.6)]
    PAS += [(43.2, sv * dv, "u") for dv in (28.1, 26.2, 23.3, 21.5)]
body, ends = [], []
for u, v, ax in PAS:
    lu, lv = (1.6, 0.8) if ax == "u" else (0.8, 1.6)
    body.append((lu, lv, 0.45, X(u), Y(v), zt + 0.225))
    for s in (-1, 1):
        if ax == "u":
            ends.append((0.3, 0.82, 0.47, X(u + s * 0.65), Y(v), zt + 0.235))
        else:
            ends.append((0.82, 0.3, 0.47, X(u), Y(v + s * 0.65), zt + 0.235))
groups["mod_copper"].append(boxes("passives", body)); groups["mod_lid"].append(boxes("passive_ends", ends))

# Cooler: embossed NVIDIA wordmark and a simplified eye mark, 0.15 mm proud, in the 12 mm band
# between the fan opening and the +y end, reading across the width.
EMB, ey = 0.15, SY1 - 6.0
eye_x = cx - 13.0
lens = cyl(3.2, top, top + EMB, eye_x, ey + 1.6, 48)                             # vesica: two discs
other = cyl(3.2, top - 1, top + 1, eye_x, ey - 1.6, 48)
m = lens.modifiers.new("i", 'BOOLEAN'); m.operation = 'INTERSECT'; m.object = other
bpy.context.view_layer.objects.active = lens; bpy.ops.object.modifier_apply(modifier=m.name); bpy.data.objects.remove(other)
lens = cut(lens, cyl(1.1, top - 1, top + 1, eye_x, ey, 32))
groups["shroud"] += [lens, cyl(0.55, top, top + EMB, eye_x, ey, 24),
                     text("NVIDIA", 3.6, cx + 2.5, ey, top, 0, "black", EMB)]

# ---- four M3 standoffs (8 mm) under the corner holes ----
for x, y in ((16.2, 100.4), (73.1, 100.4), (16.2, 6.7), (73.1, 6.7)):
    groups["silver"].append(cyl(2.5, -1.5 - 8.0, -1.5, x, y, 6))

# ---- join, weld, decimate, place, export ----
TOP_PLATE = -0.030                                    # F550 top plate surface in ft_tilt base_link (m)
place = (mathutils.Matrix.Translation((0, 0, TOP_PLATE + 0.008 + 0.0015))
         @ mathutils.Matrix.Rotation(math.radians(90), 4, 'Z')
         @ mathutils.Matrix.Scale(1e-3, 4)
         @ mathutils.Matrix.Translation((-40.0, -52.3, 0)))
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
    if n0 > 25000 and g not in ("shroud", "fan") and not g.startswith("mod_"):  # keep the modelled cooler sharp
        m = j.modifiers.new("d", 'DECIMATE'); m.ratio = 25000 / n0
        bpy.ops.object.modifier_apply(modifier=m.name)
    j.data.transform(place)
    bpy.ops.object.select_all(action='DESELECT'); j.select_set(True)
    bpy.ops.wm.stl_export(filepath=f"{outdir}/jetson_{g}.stl", export_selected_objects=True, ascii_format=False)
    vs = [v.co for v in j.data.vertices]
    print("EXPORT", g, n0, "->", len(j.data.polygons), "tris z", round(min(v[2] for v in vs), 4), round(max(v[2] for v in vs), 4))
