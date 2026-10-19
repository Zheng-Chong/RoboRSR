"""Export exactly N verified raw episodes as a local LeRobot v3 dataset.

Run in a separate environment with lerobot==0.4.4; never modifies Isaac's deps.
Nothing is uploaded to the Hub. Output must not already exist.
"""

import argparse
import json
from pathlib import Path

import numpy as np


def select_episodes(root, count):
    selected, positions = [], set()
    for path in sorted(root.rglob("quality.json")):
        quality = json.loads(path.read_text())
        if not quality["accepted"]:
            continue
        total = quality["initial_counts"]["total"]
        expected = dict(total=total, source=0, target=total, outside=0, overlap=0)
        if total <= 0 or quality["final_counts"] != expected or quality["stable_frames"] < quality["fps"]:
            raise ValueError(f"Invalid acceptance evidence: {path}")
        xy = tuple(v for label in ("source_cup", "target_cup") for v in quality["planned_poses"][label][0][:2])
        key = tuple(round(v, 5) for v in xy)
        if key in positions:
            continue
        positions.add(key)
        selected.append((path.parent, quality))
        if len(selected) == count:
            return selected
    raise ValueError(f"Only {len(selected)} unique accepted episodes; need {count}")


def validated_trajectory(folder, quality):
    with np.load(folder / "trajectory.npz") as data:
        state, action, bean_counts = data["state"], data["action"], data["counts"]
    n, fps = quality["frames"], quality["fps"]
    if state.shape != (n, 16) or action.shape != (n, 16) or not np.isfinite([state, action]).all():
        raise ValueError(f"Invalid trajectory: {folder}")
    total = quality["initial_counts"]["total"]
    expected = np.array([total, 0, total, 0, 0])
    if (
        n < fps
        or quality["stable_frames"] > n
        or bean_counts.shape != (n, 5)
        or not np.all(bean_counts[-fps:] == expected)
    ):
        raise ValueError(f"Trajectory does not retain every bean for the final second: {folder}")
    return state, action


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--episodes", type=int, default=100)
    args = parser.parse_args()
    if args.episodes < 1 or args.output.exists():
        parser.error("episodes must be positive and output must not exist")
    episodes = select_episodes(args.raw, args.episodes)

    import cv2
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    cameras = episodes[0][1]["cameras"]
    fps = episodes[0][1]["fps"]
    names = [
        f"{side}_{joint}"
        for side in ("left", "right")
        for joint in (
            "shoulder_pitch",
            "shoulder_roll",
            "shoulder_yaw",
            "elbow",
            "wrist_roll",
            "wrist_pitch",
            "wrist_yaw",
            "gripper",
        )
    ]
    features = {name: dict(dtype="float32", shape=(16,), names=names) for name in ("observation.state", "action")}
    for camera in cameras:
        cap = cv2.VideoCapture(str(episodes[0][0] / f"{camera}.mp4"))
        ok, image = cap.read()
        cap.release()
        if not ok:
            raise ValueError(f"Unreadable {camera}")
        features[f"observation.images.{camera}"] = dict(
            dtype="video", shape=image.shape, names=["height", "width", "channels"]
        )
    dataset = LeRobotDataset.create(
        "local/g1d_office_all_red_beans",
        fps=fps,
        root=args.output,
        robot_type="unitree_g1d_dex1",
        features=features,
        use_videos=True,
        vcodec="h264",
    )
    provenance = []
    try:
        for folder, quality in episodes:
            if quality["fps"] != fps or quality["cameras"] != cameras:
                raise ValueError(f"Mismatched episode schema: {folder}")
            state, action = validated_trajectory(folder, quality)
            n = quality["frames"]
            readers = {camera: cv2.VideoCapture(str(folder / f"{camera}.mp4")) for camera in cameras}
            try:
                for i in range(n):
                    frame = {"observation.state": state[i], "action": action[i], "task": quality["task"]}
                    for camera, reader in readers.items():
                        ok, rgb = reader.read()
                        if not ok:
                            raise ValueError(f"Missing frame {i}: {folder}/{camera}")
                        frame[f"observation.images.{camera}"] = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
                    dataset.add_frame(frame)
                if any(reader.read()[0] for reader in readers.values()):
                    raise ValueError(f"Extra video frames: {folder}")
                dataset.save_episode()
            finally:
                for reader in readers.values():
                    reader.release()
            provenance.append(dict(raw_directory=str(folder.resolve()), **quality))
    finally:
        dataset.finalize()
    (args.output / "collection_quality.json").write_text(json.dumps(provenance, indent=2) + "\n")
    loaded = LeRobotDataset("local/g1d_office_all_red_beans", root=args.output)
    assert loaded.num_episodes == args.episodes
    assert loaded.num_frames == sum(q["frames"] for _, q in episodes)
    for index in (0, loaded.num_frames - 1):
        frame = loaded[index]
        assert frame["observation.state"].shape == (16,)
        for camera in cameras:
            assert frame[f"observation.images.{camera}"].shape[0] == 3
    print(f"Verified {loaded.num_episodes} episodes / {loaded.num_frames} frames: {args.output}")


if __name__ == "__main__":
    main()
