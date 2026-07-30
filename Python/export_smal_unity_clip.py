"""Convert an InterPet4D SMAL sequence into a runtime clip for Unity.

The output JSON is consumed by the Unity ``SmalMotionPlayer`` component, which
drives the ``interpet_dog02_neutral_rigged`` armature bone-by-bone at runtime
(no baked animation).

Per frame we export:
  * ``deltas``   - per-joint world-space rotation delta from the neutral (rest)
                   pose, in SMAL/armature space, as quaternions (x, y, z, w).
  * ``rootRot``  - global body rotation R_world as a quaternion (x, y, z, w).
  * ``rootPos``  - global body translation t_world.
  * ``rootScale``- global body scale s_world.

Plus the constant ``restJoints`` (armature-space rest joint positions). Unity
uses ``restJoints`` to self-calibrate the SMAL->Unity coordinate map from the
imported rig, so no axis/handedness convention is hard-coded here.

The rotation-delta / rest-joint maths are shared with the animated-FBX baker
(:mod:`export_interpet_smal_animated_fbx`), which validates that applying these
deltas to the rest rig reproduces the SMAL joint positions to < 1e-4.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

# chumpy (pulled in by the SMAL model) does ``from numpy import bool, ...`` which
# was removed in NumPy 1.20+. Restore the aliases before anything imports it.
import numpy as np

for _name, _type in {
    "bool": bool,
    "int": int,
    "float": float,
    "complex": complex,
    "object": object,
    "str": str,
    "unicode": str,
    "nan": float("nan"),
    "inf": float("inf"),
}.items():
    if not hasattr(np, _name):
        setattr(np, _name, _type)

from export_interpet_smal_animated_fbx import prepare_animation  # noqa: E402
from render_interpet_smal import DEFAULT_MODEL, ROOT, require_model  # noqa: E402

SOURCE_FPS = 60


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "npz",
        type=Path,
        nargs="?",
        default=ROOT / "datasets/interpet4d/smal_npy/interpet_dog02_p04_take01_ego_001.npz",
        help="InterPet4D smal_npy .npz file",
    )
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT.parent / "Assets/SmalClips/interpet_dog02_p04_take01_ego_001.json",
    )
    parser.add_argument("--fps", type=int, default=SOURCE_FPS)
    return parser.parse_args()


def matrices_to_quaternions(matrices: np.ndarray) -> np.ndarray:
    """(..., 3, 3) proper rotations -> (..., 4) quaternions as (x, y, z, w)."""
    m = matrices.reshape(-1, 3, 3)
    quats = np.empty((m.shape[0], 4), dtype=np.float64)
    trace = m[:, 0, 0] + m[:, 1, 1] + m[:, 2, 2]

    for i in range(m.shape[0]):
        r = m[i]
        t = trace[i]
        if t > 0.0:
            s = np.sqrt(t + 1.0) * 2.0
            w = 0.25 * s
            x = (r[2, 1] - r[1, 2]) / s
            y = (r[0, 2] - r[2, 0]) / s
            z = (r[1, 0] - r[0, 1]) / s
        elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
            s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
            w = (r[2, 1] - r[1, 2]) / s
            x = 0.25 * s
            y = (r[0, 1] + r[1, 0]) / s
            z = (r[0, 2] + r[2, 0]) / s
        elif r[1, 1] > r[2, 2]:
            s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
            w = (r[0, 2] - r[2, 0]) / s
            x = (r[0, 1] + r[1, 0]) / s
            y = 0.25 * s
            z = (r[1, 2] + r[2, 1]) / s
        else:
            s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
            w = (r[1, 0] - r[0, 1]) / s
            x = (r[0, 2] + r[2, 0]) / s
            y = (r[1, 2] + r[2, 1]) / s
            z = 0.25 * s
        quats[i] = (x, y, z, w)

    quats /= np.linalg.norm(quats, axis=1, keepdims=True)
    return quats.reshape(matrices.shape[:-2] + (4,))


def main() -> None:
    args = parse_args()
    require_model(args.model)

    with np.load(args.npz) as data:
        anim = prepare_animation(data, args.model)

    rotation_deltas = anim["rotation_deltas"]          # (F, 35, 3, 3)
    world_rotations = anim["world_rotations"]          # (F, 3, 3)
    frame_count, joint_count = rotation_deltas.shape[:2]

    delta_quats = matrices_to_quaternions(rotation_deltas)     # (F, 35, 4)
    root_quats = matrices_to_quaternions(world_rotations)      # (F, 4)

    payload = {
        "name": args.npz.stem,
        "fps": int(args.fps),
        "frameCount": int(frame_count),
        "jointCount": int(joint_count),
        "boneNamePrefix": "SMAL_joint_",
        "parents": anim["parents"].astype(np.int32).tolist(),
        # armature-space rest joint positions, row-major (F rows of 3)
        "restJoints": anim["rest_joints"].reshape(-1).astype(np.float64).tolist(),
        # per-frame, per-joint quaternion deltas, flat (F * 35 * 4)
        "deltas": delta_quats.reshape(-1).tolist(),
        # per-frame global body transform
        "rootRot": root_quats.reshape(-1).tolist(),
        "rootPos": anim["world_translations"].reshape(-1).astype(np.float64).tolist(),
        "rootScale": anim["world_scales"].astype(np.float64).tolist(),
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with open(args.output, "w") as handle:
        json.dump(payload, handle, separators=(",", ":"))

    size_mb = args.output.stat().st_size / 1e6
    print("Exported Unity SMAL clip")
    print(f"Frames : {frame_count} @ {args.fps} fps")
    print(f"Joints : {joint_count}")
    print(f"Output : {args.output}  ({size_mb:.2f} MB)")


if __name__ == "__main__":
    main()
