"""White plastic mug (plastic_mug/00000) dimensions and cavity, numpy only: shared by make_plastic_cups.py (the asset)
and the bean-pour acceptance checks, so the cavity test always matches the mesh.

Measured 2026-10-19: straight 100 mm tall cylinder, 80 mm inside diameter, 2 mm wall, 89 g; handle 70 mm tall, 35 mm
out from the wall, 5 mm thick (outer extents; the handle centreline sits T/2 inside them). Bottom thickness and strap
width are guesses. Frame: Z up, metres, origin at the centre of the outside bottom, handle on +X; the asset is
shifted by origin_shift().
"""

import numpy as np

MUG = dict(
    H=0.100, R0=0.042, R1=0.042, T_WALL=0.002, T_BOTTOM=0.004, FILLET=0.003, MASS=0.089,
    HANDLE_TOP=0.0825, HANDLE_BOT=0.0175, HANDLE_OUT=0.0325, BEND=0.010, HANDLE_W=0.013, HANDLE_T=0.005,
)  # fmt: skip


def origin_shift():
    """Asset origin (bbox centre) minus this module's frame origin, negated: asset point = point + origin_shift."""
    g = MUG
    reach = g["R1"] + g["HANDLE_OUT"] + g["HANDLE_T"] / 2
    lo = np.array([-g["R0"], -g["R0"], 0.0])
    hi = np.array([reach, g["R0"], g["H"]])
    return -(lo + hi) / 2


def in_cavity(p):
    """Points (P, 3) in this module's frame strictly inside the cavity: above the inner bottom, below the rim."""
    p = np.asarray(p, float)
    inner = MUG["R0"] - MUG["T_WALL"]
    return (np.hypot(p[:, 0], p[:, 1]) < inner) & (p[:, 2] >= MUG["T_BOTTOM"]) & (p[:, 2] < MUG["H"])
