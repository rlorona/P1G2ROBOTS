# Dimensioned drawings

The Fusion 360 source is `cad/original/Proyecto_RM.f3d`. The original assembly
drawing `Proyecto_RM Dibujo_.pdf` does **not** match the exported STL geometry: several
dimensions are misplaced or refer to the wrong features. The original file was
removed from the repository for this reason, and the corrected drawings at the
project root are the authoritative ones. The STL meshes in
`ros2_ws/src/proyecto_rm_description/meshes/` are the authoritative geometry, so every
drawing must be re-exported from Fusion and checked against the values below.

## Corrected drawings

The corrected drawings delivered by the team are in `docs/drawings/`:

- `Assembly.pdf` — corrected assembly drawing (overall views, balloons, BOM).
- `BASE_LINK.pdf` — corrected chassis (`base_link`) drawing.
- `CAMERA.pdf` — corrected camera support drawing.
- `Wheel.pdf` — corrected wheel drawing.

These files are the authoritative corrected drawings to be delivered; the
original assembly drawing (`Proyecto_RM Dibujo_.pdf`) was removed from the
repository because its dimensions did not match the STL geometry.

## Sheets to export (PDF, A4 landscape, third-angle projection)

| # | Sheet | Required dimensions |
|---|-------|---------------------|
| 1 | Chassis (`base_link`) | Overall length 240 mm; overall width 180 mm; overall height 57 mm; wall thickness; positions and diameters of the wheel cut-outs; camera-mount hole pattern; base thickness |
| 2 | Camera support (`camera_1`) | 28 mm × 30 mm × 25 mm envelope; mounting-hole spacing and diameter; lens opening |
| 3 | Wheel (`*_wheel_1`) | Outside diameter 74 mm; width 22 mm; bore diameter; hub diameter; tread profile |
| 4 | Wheel assembly (wheel + axle) | Axle diameter and length; distance from the wheel centre plane to the chassis side |
| 5 | Assembly — top view | Wheelbase 160 mm; track 202 mm; overall width 224 mm; front/rear overhang; steering-knuckle pivot positions (virtual, see note) |
| 6 | Assembly — side view | Wheel radius 37 mm; axle height 37.5 mm; ground clearance; overall height |
| 7 | Assembly — isometric | Item balloons and bill of materials (chassis, camera, 4 wheels, 2 steering knuckles) |

## Notes

- All dimensions in millimetres.
- The exported STL scale is 1:1000 (Fusion exports in millimetres and the URDF
  applies `scale="0.001 0.001 0.001"`).
- The CAD has **no physical steering mechanism**. The URDF introduces two
  virtual revolute knuckles at the front-wheel centres
  (`x = -80 mm`, `y = ±101 mm`, `z = 37.5 mm`) with a ±30° range. If the CAD is
  updated later, add a real kingpin/steering arm at those pivot points.
- Do not dimension the original PDF: its values disagree with the STL. Use the
  STL-derived values in the table as the reference for every sheet. The corrected
  drawings replace it and are the ones to be delivered.
