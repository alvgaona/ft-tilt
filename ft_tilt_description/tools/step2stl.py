# STEP assembly -> one binary STL per surface colour (metres, Z up), via OpenCascade (OCP).
import sys, struct, collections
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.XCAFPrs import XCAFPrs_DocumentExplorer, XCAFPrs_DocumentExplorerFlags_OnlyLeafNodes
from OCP.RWMesh import RWMesh_FaceIterator
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.IFSelect import IFSelect_RetDone

src, outdir = sys.argv[1], sys.argv[2]
doc = TDocStd_Document(TCollection_ExtendedString("doc"))
r = STEPCAFControl_Reader(); r.SetColorMode(True); r.SetNameMode(True)
assert r.ReadFile(src) == IFSelect_RetDone, "read failed"
r.Transfer(doc)
from OCP.XCAFDoc import XCAFDoc_ShapeTool

tris = collections.defaultdict(list)
ex = XCAFPrs_DocumentExplorer(doc, XCAFPrs_DocumentExplorerFlags_OnlyLeafNodes)
while ex.More():
    n = ex.Current()
    BRepMesh_IncrementalMesh(XCAFDoc_ShapeTool.GetShape_s(n.RefLabel), 0.05, False, 0.5, True)
    it = RWMesh_FaceIterator(n.RefLabel, n.Location, True, n.Style)
    while it.More():
        if not it.IsEmptyMesh():
            c = it.FaceColor().GetRGB() if it.HasFaceColor() else None
            key = (round(c.Red(), 2), round(c.Green(), 2), round(c.Blue(), 2)) if c else (0.5, 0.5, 0.5)
            for e in range(it.ElemLower(), it.ElemUpper() + 1):
                t = it.TriangleOriented(e); a, b, cc = t.Get()
                tris[key].append(tuple(it.NodeTransformed(k) for k in (a, b, cc)))
        it.Next()
    ex.Next()

lo = [1e9] * 3; hi = [-1e9] * 3
for k, ts in tris.items():
    for t in ts:
        for p in t:
            for j, v in enumerate((p.X(), p.Y(), p.Z())):
                lo[j] = min(lo[j], v); hi[j] = max(hi[j], v)
print("BBOX mm", [round(x, 2) for x in lo], [round(x, 2) for x in hi])
for idx, (k, ts) in enumerate(sorted(tris.items(), key=lambda kv: -len(kv[1]))):
    name = f"{outdir}/part_{idx:02d}.stl"
    with open(name, "wb") as f:
        f.write(b"\0" * 80 + struct.pack("<I", len(ts)))
        for t in ts:
            f.write(struct.pack("<3f", 0, 0, 0))
            for p in t:
                f.write(struct.pack("<3f", p.X() * 1e-3, p.Y() * 1e-3, p.Z() * 1e-3))
            f.write(b"\0\0")
    print("COLOUR", idx, k, len(ts), "tris")
