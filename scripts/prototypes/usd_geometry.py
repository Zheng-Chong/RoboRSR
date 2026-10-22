"""Write plain USD geometry; author no rigid, deformable, collision or tactile APIs."""

import math
from pathlib import Path


def write_mesh(path, points, faces, *, color=(0.38, 0.42, 0.46)):
    points = [tuple(float(v) for v in p) for p in points]
    faces = [tuple(f) for f in faces]
    if not points or not faces or any(len(p) != 3 or not all(map(math.isfinite, p)) for p in points):
        raise ValueError("Expected finite 3D points and nonempty triangle topology")
    if any(len(f) != 3 or any(type(i) is not int or not 0 <= i < len(points) for i in f) for f in faces):
        raise ValueError("Invalid triangle indices")
    lo = tuple(min(p[i] for p in points) for i in range(3))
    hi = tuple(max(p[i] for p in points) for i in range(3))
    with Path(path).open("w") as out:
        out.write('#usda 1.0\n(defaultPrim = "Prototype"\n metersPerUnit = 1\n upAxis = "Z")\n')
        out.write('def Xform "Prototype" {\n def Mesh "geometry" {\n')
        out.write('  uniform token subdivisionScheme = "none"\n  uniform bool doubleSided = true\n')
        out.write('  custom string capability = "geometry_reference_only"\n')
        out.write("  point3f[] points = [\n")
        for p in points:
            out.write(f"   {p},\n")
        out.write("  ]\n  int[] faceVertexCounts = [" + ",".join("3" for _ in faces) + "]\n")
        out.write("  int[] faceVertexIndices = [" + ",".join(str(i) for f in faces for i in f) + "]\n")
        out.write(f"  float3[] extent = [{lo}, {hi}]\n")
        out.write(f"  color3f[] primvars:displayColor = [{tuple(color)}]\n }}\n}}\n")
    return dict(vertices=len(points), triangles=len(faces), bounds_m=[lo, hi])
