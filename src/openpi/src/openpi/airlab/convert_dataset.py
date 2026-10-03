"""Convert Panda H5 episodes to local LeRobot data using custom velocity actions only.

Run from the OpenPI project root:
    uv run src/openpi/airlab/convert_dataset.py --data-dir ../../data/episodes

Output defaults to ../lerobot next to the input episodes directory.
Arm action: (q[t+1] - q[t]) / (timestamps[t+1] - timestamps[t]) [rad/s].
Gripper state/action: 1 - observed/commanded width / 0.08 (0=open, 1=closed).
The final frame is omitted because it has no next position for the difference.
"""

from pathlib import Path
import warnings

import numpy as np

REPO_ID = "airlab/pick_place"
DEFAULT_DATA_DIR = Path(__file__).resolve().parents[5] / "data" / "episodes"
ARM_JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
MAX_GRIPPER_WIDTH = 0.08
IMAGE_FIELDS = {"agent_image": "exterior_image_1_left", "wrist_image": "wrist_image_left"}


def text(value):
    return value.decode("utf-8") if isinstance(value, bytes) else str(value)


def read_episode(episode):
    """Validate one H5 episode and prepare numeric data without loading all images."""
    required = ["timestamps", "joint_positions", "gripper_width", "action_gripper_width", *IMAGE_FIELDS]
    missing = [key for key in required if key not in episode]
    if missing:
        raise ValueError(f"{episode.filename}: missing fields {missing}")
    for key in ("joint_names", "image_encoding", "instruction"):
        if key not in episode.attrs:
            raise ValueError(f"{episode.filename}: missing attribute {key}")

    timestamps = np.asarray(episode["timestamps"], dtype=np.float64)
    positions = np.asarray(episode["joint_positions"], dtype=np.float64)
    width = np.asarray(episode["gripper_width"], dtype=np.float64)
    commanded_width = np.asarray(episode["action_gripper_width"], dtype=np.float64)
    if timestamps.ndim != 1 or len(timestamps) < 2:
        raise ValueError(f"{episode.filename}: at least two timestamps are required")
    n = len(timestamps)
    joint_names = [text(name) for name in episode.attrs["joint_names"]]
    if len(set(joint_names)) != len(joint_names) or any(name not in joint_names for name in ARM_JOINTS):
        raise ValueError(f"{episode.filename}: joint_names must contain each Panda arm joint exactly once")
    if positions.shape != (n, len(joint_names)) or width.shape != (n,) or commanded_width.shape != (n,):
        raise ValueError(f"{episode.filename}: inconsistent joint/width shapes or frame counts")
    if not all(np.isfinite(values).all() for values in (timestamps, positions, width, commanded_width)):
        raise ValueError(f"{episode.filename}: non-finite timestamps, positions or widths")
    dt = np.diff(timestamps)
    if np.any(dt <= 0):
        raise ValueError(f"{episode.filename}: timestamps must be strictly increasing")

    encoding = text(episode.attrs["image_encoding"])
    if encoding not in ("bgr8", "rgb8"):
        raise ValueError(f"{episode.filename}: unsupported image encoding {encoding}")
    image_shapes = {}
    for key, target in IMAGE_FIELDS.items():
        images = episode[key]
        if images.ndim != 4 or images.shape[0] != n or images.shape[-1] != 3 or images.dtype != np.uint8:
            raise ValueError(f"{episode.filename}: {key} must be uint8 [N, H, W, 3]")
        if min(images.shape[1:3]) <= 0:
            raise ValueError(f"{episode.filename}: {key} has an empty image")
        image_shapes[target] = images.shape[1:]
    instruction = text(episode.attrs["instruction"]).strip()
    if not instruction:
        raise ValueError(f"{episode.filename}: empty instruction")

    joints = positions[:, [joint_names.index(name) for name in ARM_JOINTS]]
    velocity = np.diff(joints, axis=0) / dt[:, None]
    gripper = np.clip(1.0 - width / MAX_GRIPPER_WIDTH, 0.0, 1.0)
    gripper_action = np.clip(1.0 - commanded_width[:-1] / MAX_GRIPPER_WIDTH, 0.0, 1.0)
    actions = np.column_stack((velocity, gripper_action)).astype(np.float32)
    joints = joints[:-1].astype(np.float32)
    if not np.isfinite(actions).all() or not np.isfinite(joints).all():
        raise ValueError(f"{episode.filename}: joint values or velocities overflow float32")
    return joints, gripper[:-1, None].astype(np.float32), actions, instruction, encoding, image_shapes, dt


def main(data_dir: Path = DEFAULT_DATA_DIR, *, output_dir: Path | None = None, fps: int = 15):
    import h5py
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

    data_dir = data_dir.expanduser().resolve()
    output_dir = (output_dir.expanduser() if output_dir is not None else data_dir.parent / "lerobot").resolve()
    if fps <= 0:
        raise ValueError("fps must be positive")
    if not data_dir.is_dir():
        raise FileNotFoundError(f"Input directory does not exist: {data_dir}")
    if output_dir.exists():
        raise FileExistsError(f"Output already exists: {output_dir}. Use a new --output-dir; nothing was deleted.")
    paths = sorted(data_dir.glob("*.h5"))
    if not paths:
        raise ValueError(f"No H5 episodes found in {data_dir}")

    # Validate every episode before creating the output directory.
    image_shapes = None
    for path in paths:
        with h5py.File(path, "r") as episode:
            *_, shapes, dt = read_episode(episode)
            if image_shapes is not None and shapes != image_shapes:
                raise ValueError(f"{path}: camera image sizes differ between episodes")
            image_shapes = shapes
            if abs(1.0 / np.median(dt) - fps) > 0.1 * fps or np.max(dt) > 2.0 / fps:
                warnings.warn(f"{path.name}: irregular sampling or fps mismatch; no temporal resampling is applied.")

    features = {
        key: {"dtype": "image", "shape": shape, "names": ["height", "width", "channel"]}
        for key, shape in image_shapes.items()
    }
    features.update({
        "joint_position": {"dtype": "float32", "shape": (7,), "names": ["joint_position"]},
        "gripper_position": {"dtype": "float32", "shape": (1,), "names": ["gripper_position"]},
        "actions": {"dtype": "float32", "shape": (8,), "names": ["actions"]},
    })
    dataset = LeRobotDataset.create(
        repo_id=REPO_ID, root=output_dir, robot_type="panda", fps=fps, features=features,
        image_writer_threads=4, image_writer_processes=0,
    )
    for path in paths:
        with h5py.File(path, "r") as episode:
            joints, gripper, actions, instruction, encoding, _, _ = read_episode(episode)
            for i in range(len(actions)):
                frame = {"joint_position": joints[i], "gripper_position": gripper[i],
                         "actions": actions[i], "task": instruction}
                for key, target in IMAGE_FIELDS.items():
                    image = episode[key][i]
                    frame[target] = np.ascontiguousarray(image[..., ::-1] if encoding == "bgr8" else image)
                dataset.add_frame(frame)
            dataset.save_episode()
            print(f"{path.name}: {len(joints) + 1} -> {len(joints)} frames")
    print(f"Saved {len(paths)} episodes: {output_dir}")


if __name__ == "__main__":
    import tyro

    tyro.cli(main)
