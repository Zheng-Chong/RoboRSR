"""Convert a hash-pinned vendor STEP into local USD/MJCF visual references.

Use --step FILE or --download. Requires cadquery-ocp==7.7.2 in an isolated
CAD environment. No vendor CAD, derived geometry or screenshots are bundled.
The CAD origin is preserved; no mounting frame or moving links are inferred.
"""

import argparse
import hashlib
import json
from pathlib import Path
import struct
import tempfile
import urllib.request

from usd_geometry import write_mesh

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "real2sim/prototypes/aurora_lite/source.json"


def verify_source(path, spec):
    data = Path(path).read_bytes()
    if len(data) != spec["source_bytes"] or hashlib.sha256(data).hexdigest() != spec["source_sha256"]:
        raise ValueError("Vendor STEP differs from pinned size/hash; review upstream changes before conversion")
    if not data.rstrip().endswith(b"END-ISO-10303-21;") or b"SI_UNIT ( .MILLI., .METRE. )" not in data:
        raise ValueError("Incomplete STEP or unrecognized units")


def read_stl(path):
    data = Path(path).read_bytes()
    if len(data) < 84:
        raise ValueError("Truncated binary STL")
    count = struct.unpack_from("<I", data, 80)[0]
    if count == 0 or len(data) != 84 + 50 * count:
        raise ValueError("Invalid binary STL triangle count")
    vertices, faces, ids = [], [], {}
    for record in struct.iter_unpack("<12fH", data[84:]):
        face = []
        for i in (3, 6, 9):
            point = tuple(record[i : i + 3])
            if point not in ids:
                ids[point] = len(vertices)
                vertices.append(tuple(v * 0.001 for v in point))
            face.append(ids[point])
        faces.append(tuple(face))
    return vertices, faces, data


def export_visual(stl, output):
    output = Path(output)
    vertices, faces, data = read_stl(stl)
    report = write_mesh(output / "geometry.usda", vertices, faces)
    mesh = output / "meshes"
    mesh.mkdir()
    count = len(faces)
    assets, geoms = [], []
    # MuJoCo limits STL triangle count. Chunks retain CAD coordinates, not links.
    for part, start in enumerate(range(0, count, 150000)):
        n = min(150000, count - start)
        file = mesh / f"part_{part:03}.stl"
        file.write_bytes(data[:80] + struct.pack("<I", n) + data[84 + start * 50 : 84 + (start + n) * 50])
        assets.append(f'<mesh name="part_{part}" file="meshes/{file.name}" scale=".001 .001 .001"/>')
        geoms.append(f'<geom type="mesh" mesh="part_{part}" contype="0" conaffinity="0" mass="0"/>')
    xml = '<mujoco model="aurora_lite_static_reference"><asset>' + "".join(assets)
    xml += '</asset><worldbody><body name="vendor_cad_origin">' + "".join(geoms)
    xml += "</body></worldbody></mujoco>\n"
    (output / "reference.xml").write_text(xml)
    return dict(
        **report,
        mesh_chunks=len(assets),
        cad_to_metre_scale=0.001,
        articulated=False,
        collision_enabled=False,
        tactile_simulated=False,
        mounting_frame_verified=False,
        cad_origin_preserved=True,
    )


def convert(step, output, spec):
    verify_source(step, spec)
    # Lazy imports keep source/hash and geometry tests independent of CAD packages.
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPControl import STEPControl_Reader
    from OCP.StlAPI import StlAPI_Writer

    reader = STEPControl_Reader()
    if reader.ReadFile(str(step)) != IFSelect_RetDone:
        raise ValueError("STEP reader failed")
    reader.TransferRoots()
    shape = reader.OneShape()
    tessellation = BRepMesh_IncrementalMesh(shape, 0.25, False, 0.3, True)
    if not tessellation.IsDone():
        raise ValueError("STEP tessellation failed")
    from OCP.Bnd import Bnd_Box
    from OCP.BRep import BRep_Tool
    from OCP.BRepBndLib import BRepBndLib
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS

    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, box, False, False)
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    missing_faces, missing_area = 0, 0.0
    while explorer.More():
        face = TopoDS.Face_s(explorer.Current())
        triangulation = BRep_Tool.Triangulation_s(face, TopLoc_Location())
        if triangulation is None or triangulation.NbTriangles() == 0:
            missing_faces += 1
            properties = GProp_GProps()
            BRepGProp.SurfaceProperties_s(face, properties)
            missing_area += properties.Mass()
        explorer.Next()
    with tempfile.TemporaryDirectory() as temp:
        stl = Path(temp) / "visual_mm.stl"
        writer = StlAPI_Writer()
        writer.ASCIIMode = False
        if not writer.Write(shape, str(stl)):
            raise ValueError("STL export failed")
        report = export_visual(stl, output)
    report.update(
        source_exact_bounds_mm=list(box.Get()),
        untriangulated_cad_faces=missing_faces,
        untriangulated_cad_area_mm2=missing_area,
        cad_surface_complete=missing_faces == 0,
        source_sha256=spec["source_sha256"],
        source_url=spec["source_url"],
        tessellation_linear_deflection_mm=0.25,
        tessellation_angular_deflection_rad=0.3,
    )
    (output / "conversion.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--step", type=Path)
    source.add_argument("--download", action="store_true", help="Fetch official, pinned 80 MB CAD for local conversion")
    parser.add_argument("--output", type=Path, required=True, help="New local directory; never overwritten")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Output already exists; use a new directory")
    spec = json.loads(SOURCE.read_text())
    args.output.mkdir(parents=True, exist_ok=False)
    with tempfile.TemporaryDirectory() as temp:
        step = args.step
        if args.download:
            step = Path(temp) / "aurora-lite-gripper.STEP"
            with urllib.request.urlopen(spec["source_url"], timeout=60) as response, step.open("wb") as out:
                remaining = spec["source_bytes"] + 1
                while remaining > 0:
                    chunk = response.read(min(1024 * 1024, remaining))
                    if not chunk:
                        break
                    out.write(chunk)
                    remaining -= len(chunk)
        convert(step, args.output, spec)


if __name__ == "__main__":
    main()
