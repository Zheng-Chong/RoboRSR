"""ChArUco board for real-scene capture: print a board, then recover each photo's camera pose in metres from it.

With a board of known size lying on the table in the photos, a scene fit no longer hangs on one size prior (such as
"the table is 0.75 m high"), and the phone needs no separate calibration: the board photos calibrate it.

Usage (opencv-python >= 4.7, numpy):
    python scripts/scenes/charuco.py board --out board.png      # 7 x 5 squares, fits A4 at 300 dpi
    python scripts/scenes/charuco.py solve photos/*.jpg --square-mm 35.9 --out cameras.json [--intrinsics K.json]
Measure one printed square with calipers and pass that as --square-mm: printers scale. cameras.json holds the
intrinsics (calibrated from the photos unless --intrinsics gives them; at least 3 views of the board) and, per photo,
camera_from_board (4x4, OpenCV camera axes), the camera position in the board frame and the reprojection RMS. Board
frame: origin at the printed board's outer top-left corner, x along the 7 squares, y along the 5, z into the table.
"""

import argparse
import json
from pathlib import Path

import numpy as np

SQUARES, SQUARE, MARKER, DICTIONARY = (7, 5), 0.036, 0.027, "DICT_5X5_100"
MIN_CORNERS = 6


def make_board(squares=SQUARES, square=SQUARE, marker=MARKER, dictionary=DICTIONARY):
    import cv2

    return cv2.aruco.CharucoBoard(
        squares, square, marker, cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
    )


def detect(image, board):
    """(object points (N, 3) m, image points (N, 2) px) of the board corners seen in a BGR or grey image; None if
    fewer than MIN_CORNERS."""
    import cv2

    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, _, _ = cv2.aruco.CharucoDetector(board).detectBoard(gray)
    if ids is None or len(ids) < MIN_CORNERS:
        return None
    obj, img = board.matchImagePoints(corners, ids)
    return obj.reshape(-1, 3).astype(np.float64), img.reshape(-1, 2).astype(np.float64)


def calibrate(views, size):
    """K, distortion and RMS (px) from >= 3 views [(obj, img)] of the flat board; tangential and k3 fixed at 0, which
    a few phone photos cannot pin down."""
    import cv2

    if len(views) < 3:
        raise ValueError(f"{len(views)} views of the board; calibration needs 3 or more")
    flags = cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3
    obj = [o.astype(np.float32) for o, _ in views]
    img = [i.astype(np.float32) for _, i in views]
    rms, K, dist, _, _ = cv2.calibrateCamera(obj, img, size, None, None, flags=flags)
    return K, dist.reshape(-1), float(rms)


def board_pose(obj, img, K, dist):
    """camera_from_board 4x4 and the reprojection RMS (px)."""
    import cv2

    ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist)
    if not ok:
        raise ValueError("solvePnP failed")
    projected, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = cv2.Rodrigues(rvec)[0], tvec.reshape(-1)
    return T, float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - img) ** 2, axis=1))))


def solve(paths, board, intrinsics=None):
    import cv2

    images = {str(p): cv2.imread(str(p)) for p in paths}
    sizes = {im.shape[1::-1] for im in images.values() if im is not None}
    if len(sizes) != 1:
        raise ValueError(f"Photos must be readable and the same size, got {sizes}")
    views = {p: v for p, im in images.items() if (v := detect(im, board)) is not None}
    if intrinsics:
        K, dist, rms = np.array(intrinsics["K"]), np.array(intrinsics["dist"]), None
    else:
        K, dist, rms = calibrate(list(views.values()), sizes.pop())
    result = dict(K=K.tolist(), dist=dist.tolist(), calibration_rms_px=rms, photos={})
    for path in images:
        if path not in views:
            result["photos"][path] = None
            continue
        T, err = board_pose(*views[path], K, dist)
        camera_in_board = np.linalg.inv(T)[:3, 3]
        result["photos"][path] = dict(
            corners=len(views[path][0]),
            camera_from_board=T.round(6).tolist(),
            camera_in_board_m=camera_in_board.round(4).tolist(),
            rms_px=round(err, 3),
        )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    printing = sub.add_parser("board", help="write a printable board image")
    printing.add_argument("--out", type=Path, required=True)
    printing.add_argument("--dpi", type=int, default=300)
    solving = sub.add_parser("solve", help="camera poses from photos of the board")
    solving.add_argument("photos", nargs="+", type=Path)
    solving.add_argument("--square-mm", type=float, required=True, help="measured printed square size")
    solving.add_argument("--intrinsics", type=Path, help="JSON with K and dist; default: calibrate from the photos")
    solving.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "board":
        import cv2

        px = round(SQUARE / 0.0254 * args.dpi)
        image = make_board().generateImage((SQUARES[0] * px, SQUARES[1] * px), marginSize=0, borderBits=1)
        cv2.imwrite(str(args.out), image)
        mm = px / args.dpi * 25.4
        print(f"{args.out}: {SQUARES[0]} x {SQUARES[1]} squares of {mm:.2f} mm at {args.dpi} dpi; print at 100 %")
        return
    board = make_board(square=args.square_mm / 1000, marker=args.square_mm / 1000 * MARKER / SQUARE)
    intrinsics = json.loads(args.intrinsics.read_text()) if args.intrinsics else None
    result = solve(args.photos, board, intrinsics)
    args.out.write_text(json.dumps(result, indent=2) + "\n")
    found = sum(v is not None for v in result["photos"].values())
    print(f"{found}/{len(result['photos'])} photos with the board; calibration RMS {result['calibration_rms_px']}")


if __name__ == "__main__":
    main()
