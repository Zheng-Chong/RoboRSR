"""Generate standalone, uncalibrated textile topology fixtures (SI units)."""

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class ClothConfig:
    width_m: float = 0.16
    length_m: float = 0.16
    nx: int = 9
    ny: int = 9
    areal_density_kg_m2: float = 0.2
    thickness_m: float = 0.001
    height_m: float = 0.1
    friction: float = 0.5
    edge_damping: float = 0.01
    timestep_s: float = 0.0002

    def validate(self):
        for key, value in asdict(self).items():
            if isinstance(value, bool) or not math.isfinite(value):
                raise ValueError(f"{key} must be finite and numeric")
            if value < 0 or (value == 0 and key not in ("friction", "edge_damping")):
                raise ValueError(f"{key} must be positive (friction/damping may be zero)")
        for key in ("nx", "ny"):
            value = getattr(self, key)
            if not isinstance(value, int) or not 2 <= value <= 101:
                raise ValueError(f"{key} must be an integer in [2, 101]")
        spacing = min(self.width_m / (self.nx - 1), self.length_m / (self.ny - 1))
        if self.thickness_m >= spacing:
            raise ValueError("thickness_m must be smaller than grid spacing")
        if self.height_m <= self.thickness_m / 2:
            raise ValueError("height_m must place the cloth above the floor")


def build(config):
    config.validate()
    mass = config.width_m * config.length_m * config.areal_density_kg_m2
    root = ET.Element("mujoco", model="cloth_patch_prototype_v1")
    ET.SubElement(root, "option", timestep=str(config.timestep_s), integrator="implicitfast")
    world = ET.SubElement(root, "worldbody")
    contact = dict(friction=f"{config.friction} .005 .0001", solref=".001 1", solimp=".99 .999 .001")
    ET.SubElement(world, "geom", name="floor", type="plane", size="1 1 .01", **contact)
    flex = ET.SubElement(
        world,
        "flexcomp",
        name="cloth",
        type="grid",
        dim="2",
        count=f"{config.nx} {config.ny} 1",
        spacing=f"{config.width_m / (config.nx - 1)} {config.length_m / (config.ny - 1)} 1",
        mass=str(mass),
        radius=str(config.thickness_m / 2),
        pos=f"0 0 {config.height_m}",
        rgba=".3 .6 .8 1",
    )
    ET.SubElement(
        flex, "edge", equality="true", damping=str(config.edge_damping), solref=".001 1", solimp=".99 .999 .001"
    )
    ET.SubElement(flex, "contact", selfcollide="auto", **contact)
    ET.indent(root)
    xml = ET.tostring(root, encoding="unicode") + "\n"
    metadata = dict(
        schema="robodojo.textile_fixture.v1",
        id="cloth_patch",
        version="v1",
        kind="standalone_deformable_test_fixture",
        model="model.xml",
        status="prototype_not_real_calibrated",
        units="m,kg,s",
        config=asdict(config),
        mass_kg=mass,
        model_sha256=hashlib.sha256(xml.encode()).hexdigest(),
        constitutive_model="triangulated edge-length constraints; no calibrated bending response",
        material_axes={"warp": "+x", "weft": "+y", "anisotropy_implemented": False},
        real_identification={
            key: None
            for key in (
                "warp_stretch",
                "weft_stretch",
                "shear",
                "bending",
                "damping",
                "cloth_cloth_static_dynamic_friction",
                "cloth_pad_static_dynamic_friction",
            )
        },
        real_material_calibrated=False,
        robot_integrated=False,
        tactile_sensor_integrated=False,
    )
    return xml, metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, help="JSON object of ClothConfig fields")
    parser.add_argument("--output", type=Path, required=True, help="New asset directory")
    args = parser.parse_args()
    try:
        config = ClothConfig(**json.loads(args.config.read_text())) if args.config else ClothConfig()
        xml, metadata = build(config)
    except (TypeError, ValueError) as exc:
        parser.error(str(exc))
    # Never silently replace an existing asset version.
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "model.xml").write_text(xml)
    (args.output / "asset.json").write_text(json.dumps(metadata, indent=2) + "\n")
    from usd_geometry import write_mesh

    points = [
        (
            i * config.width_m / (config.nx - 1) - config.width_m / 2,
            j * config.length_m / (config.ny - 1) - config.length_m / 2,
            0.0,
        )
        for j in range(config.ny)
        for i in range(config.nx)
    ]
    faces = []
    for j in range(config.ny - 1):
        for i in range(config.nx - 1):
            a = j * config.nx + i
            faces.extend([(a, a + 1, a + config.nx), (a + 1, a + config.nx + 1, a + config.nx)])
    write_mesh(args.output / "geometry.usda", points, faces, color=(0.3, 0.6, 0.8))
    instance = {
        "physics": {
            "garment_config": {
                "particle_mass": metadata["mass_kg"] / len(points),
                "stretch_stiffness": 1000.0,
                "bend_stiffness": 0.001,
                "shear_stiffness": 100.0,
                "spring_damping": 0.1,
            },
            "particle_material": {"friction": config.friction, "adhesion": 0.0, "particle_adhesion_scale": 0.0},
            "particle_system": {
                "contact_offset": 0.0015,
                "rest_offset": 0.0005,
                "particle_contact_offset": 0.0015,
                "solid_rest_offset": 0.0005,
                "fluid_rest_offset": 0.0005,
            },
        },
        "visual": {"color": [0.3, 0.6, 0.8]},
        "calibration": {
            "status": "uncalibrated_solver_candidate",
            "backend": "isaac_sim_particle_cloth",
            "mujoco_material_equivalence_verified": False,
        },
    }
    (args.output / "isaac_instance.json").write_text(json.dumps(instance, indent=2) + "\n")


if __name__ == "__main__":
    main()
