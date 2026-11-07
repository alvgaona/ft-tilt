# Mesh tools

Scripts that generate the visual meshes in `../meshes`. Run from this directory, Blender ≥ 4.2
(`B` = path to the Blender binary). Visual only; vehicle dynamics come from the xacro numbers.

| Script | Makes | Command |
|---|---|---|
| `f550_export.py` | `f550_*.stl` from the Printables 1436460 F550 STL | `B -b -P f550_export.py -- f550.stl ../meshes` |
| `a2212.py` | `motor_*.stl` (A2212 920KV look) | `B -b -P a2212.py -- ../meshes` |
| `adapter.py` | `adapter.stl` (17° printed tilt wedge, RPIC17 Fig. 4) | `B -b -P adapter.py -- ../meshes` |
| `step2stl.py` | one STL per colour from NVIDIA's dev kit STEP (minus the tray and antennas), plus `colours.txt` | `pixi exec -s ocp -s python=3.11 python step2stl.py P3766-P3768SKU4-P3767ENVELOPE.stp devkit/ 375-0137-000 P3526_WIFI_ANT_LEFT P3526_WIFI_ANT_MODULE > devkit/colours.txt` |
| `jetson_build.py` | `jetson_*.stl` (carrier, module, cooler, standoffs) | `B -b -P jetson_build.py -- devkit devkit/colours.txt ../meshes` |
| `open_jetson.py` | opens the Jetson meshes in Blender, grouped in collections | `B -P open_jetson.py [-- jetson.blend]` |

Inputs (F550 STL, dev kit STEP) are not in the repo. The STEP is NVIDIA's Jetson Orin Nano
developer kit model (`P3766-P3768SKU4-P3767ENVELOPE.stp`).
