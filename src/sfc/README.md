# sfc
THe safety corridor generating tools implemented by Sikang LIU. I modify the function for checking if one point `pt` is inside the polyhedron `poly`, add a parameter for safety margin `ds`. If a point's distance to any plane of the polyhedron is smaller than the margin, the function return false (`poly.inside(pt,ds)` returns `false`).
