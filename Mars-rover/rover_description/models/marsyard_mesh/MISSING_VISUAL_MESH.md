# marsyard2026_visual.obj is not in this repo

It's 276MB — over GitHub's hard 100MB-per-file push limit, so it's excluded
here. `marsyard2026_collision.obj` (used for physics) is present; this one
is the textured visual mesh Gazebo renders, referenced by
`rover_description/worlds/marsyard.sdf`.

Get it from wherever the rest of the Mars Yard 2026 source data lives (it
was originally provided alongside `Model3D_mesh1.ply` /
`Coordinates_MarsYard2026.txt`, which *are* included under
`marsyard/2026_MarsYard_3D_Model-20260812T165935Z-1-001/`), or ask whoever
generated it. Drop it in this same directory once you have it —
`marsyard.sdf` expects it at exactly this path.
