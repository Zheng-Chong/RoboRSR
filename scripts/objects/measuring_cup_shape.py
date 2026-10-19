"""Measuring-cup dimensions and cross-sections, numpy only: shared by make_plastic_cups.py (the asset) and the bean
counter of scripts/g1d/collect_office_beans.py, so the cavity test always matches the mesh.

Measured 2026-10-19 (both cups): 2 mm wall, 75 mm across the flats at the mouth (the lid-seat flange, the widest
part), 63 mm at the bottom; tall 170 mm / 60 g, short 140 mm / 40 g. Sections from the top-view photos: the bottom is
a square with 45-deg chamfered corners (rounded octagon), the mouth a rounded square. The body is a ruled surface
between the two up to the flange; above it a straight snap-lid band runs to the rim.

Frame: Z up, metres, origin at the centre of the outside bottom; the asset is shifted by origin_shift().
"""

import numpy as np

N_THETA = 192  # points per ring; multiple of 8 so rings of both sections line up corner to corner
# half-widths at face centre: outside bottom, body under the flange, lid band, flange
CUP = dict(
    A0=0.0315, A1=0.0342, A_COLLAR=0.035, A_LIP=0.0375,
    COLLAR_H=0.0125, LIP_T=0.002, T_WALL=0.002, T_BOTTOM=0.002, FILLET=0.004,
)  # fmt: skip
MEASURING = [(0.14, 400, 0.040), (0.17, 500, 0.060)]  # height, top graduation (ml), mass; index 0 short, 1 tall
TAB_OUT, TAB_W = 0.007, 0.012  # small hanging tab on the flange at +X: reach past the flange, width


def rounded_octagon(chamfer, radius):
    """Square of half-width 1, corners cut at 45 deg (leg `chamfer`) and then rounded (`radius`), both as fractions
    of the half-width; N_THETA points uniform by arc length, CCW from the +X face centre."""
    w, c = 1 - radius, chamfer
    corner = np.array([(w, w - c), (w - c, w)])
    verts = np.concatenate([_rot(corner, k) for k in range(4)])  # 8 vertices CCW; vertex k owns normals 45k..45k+45
    phi = np.linspace(0, np.pi / 4, 64)
    curve = np.concatenate(
        [v + radius * np.c_[np.cos(phi + k * np.pi / 4), np.sin(phi + k * np.pi / 4)] for k, v in enumerate(verts)]
    )
    closed = np.vstack([curve, curve[:1]])
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(closed, axis=0), axis=1))]
    u = (s[-1] - (w - c) + np.linspace(0, s[-1], N_THETA, endpoint=False)) % s[-1]  # (1, 0) is w - c before curve[0]
    return np.c_[np.interp(u, s, closed[:, 0]), np.interp(u, s, closed[:, 1])]


def _rot(p, k):
    c, s = np.cos(k * np.pi / 2), np.sin(k * np.pi / 2)
    return np.rint(p @ np.array([[c, s], [-s, c]]) * 1e12) / 1e12


BOTTOM = rounded_octagon(0.30, 0.06)
TOP = rounded_octagon(0.0, 0.30)


def flange_z(H):
    return H - CUP["COLLAR_H"]


def _half_width(z, H):
    """Face-centre half-width of the body (extrapolated above the flange) and the section blend, broadcastable."""
    t = np.asarray(z, float)[..., None, None] / flange_z(H)
    return CUP["A0"] + (CUP["A1"] - CUP["A0"]) * t, np.minimum(t, 1)


def outer_ring(z, H):
    """Outside of the body at height z, shape (..., N_THETA, 2)."""
    a, s = _half_width(z, H)
    a0, a1 = CUP["A0"], CUP["A1"]
    return a * ((1 - s) * a0 * BOTTOM + s * a1 * TOP) / (a0 + (a1 - a0) * s)


def inner_ring(z, H):
    """Inner wall at height z (continues the body taper up through the lid band to the rim)."""
    a, _ = _half_width(z, H)
    return outer_ring(z, H) * (a - CUP["T_WALL"]) / a


def origin_shift(H):
    """Asset origin (bbox centre) minus this module's frame origin, negated: asset point = point + origin_shift."""
    lo = np.array([-CUP["A_LIP"], -CUP["A_LIP"], 0.0])
    hi = np.array([CUP["A_LIP"] + TAB_OUT, CUP["A_LIP"], H])
    return -(lo + hi) / 2


def in_cavity(p, H):
    """Points (P, 3) in this module's frame strictly inside the cavity: above the inner bottom, below the rim."""
    p = np.asarray(p, float)
    ring = inner_ring(p[:, 2], H)
    e, d = np.roll(ring, -1, axis=-2) - ring, p[:, None, :2] - ring
    inside = np.all(e[..., 0] * d[..., 1] - e[..., 1] * d[..., 0] > 0, axis=-1)
    return inside & (p[:, 2] >= CUP["T_BOTTOM"]) & (p[:, 2] < H)
