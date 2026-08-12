"""Export an InterPet4D SMAL sequence as a *rig-free* Unity clip.

This is the sibling of :mod:`export_smal_unity_clip`, but instead of leaving the
joints in raw SMAL/armature space (and relying on Unity to self-calibrate the
SMAL->Unity coordinate map from an imported rig), it BAKES the SMAL->Unity
coordinate conversion into every position and rotation. The resulting clip is
already expressed in Unity's left-handed, Y-up convention, so it can be played
with **no imported FBX rig and no runtime calibration** — see the Unity
``SmalMotionPlayerRigFree`` component, which builds its own joint skeleton from
this clip.

Why a conversion is needed at all: SMAL is right-handed, Z-up; Unity is
left-handed, Y-up. The two differ by a reflection (a handedness flip), which a
rotation-only retarget cannot cancel — omit it and the legs come out mirrored
(flipped ~180 deg). The conversion below is the fixed change of basis

    C = | 1 0 0 |    (swap Y and Z; det(C) = -1)
        | 0 0 1 |
        | 0 1 0 |

applied to points as ``p' = C p`` and to rotations as ``R' = C R C^-1`` (C is a
symmetric involution, so ``C^-1 = C``). Because the retarget builds a torso frame
and normalizes by bone length, the exact resting orientation is irrelevant — only
the handedness and rough up-axis matter — so this single fixed conversion is
enough. If a particular export still comes out mirrored or upside down, flip the
sign of one column of C (or rotate the player's GameObject).

Output JSON is the same schema as :mod:`export_smal_unity_clip` plus a
``"space":"unity"`` marker; the extra field is ignored by the original
``SmalMotionPlayer`` and used only as documentation here.
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

# Reuse the rotation->quaternion maths and the animation prep from the existing
# scripts (imported, never modified).
from export_smal_unity_clip import matrices_to_quaternions  # noqa: E402
from export_interpet_smal_animated_fbx import prepare_animation  # noqa: E402
from render_interpet_smal import DEFAULT_MODEL, ROOT, require_model  # noqa: E402

SOURCE_FPS = 60

# SMAL (right-handed, Z-up) -> Unity (left-handed, Y-up): swap Y and Z. det = -1,
# i.e. this is the handedness flip a rotation-only retarget cannot otherwise undo.
SMAL_TO_UNITY = np.array(
    [[1.0, 0.0, 0.0],
     [0.0, 0.0, 1.0],
     [0.0, 1.0, 0.0]],
    dtype=np.float64,
)


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
        default=ROOT.parent / "Assets/SmalClips/interpet_dog02_p04_take01_ego_001_rigfree.json",
    )
    parser.add_argument("--fps", type=int, default=SOURCE_FPS)
    return parser.parse_args()


def convert_points(points: np.ndarray) -> np.ndarray:
    """Apply the SMAL->Unity change of basis to (..., 3) points: p' = C p."""
    return points @ SMAL_TO_UNITY.T


def convert_rotations(matrices: np.ndarray) -> np.ndarray:
    """Conjugate (..., 3, 3) rotations into the Unity basis: R' = C R C^-1.

    C is a symmetric involution (C^-1 = C.T = C), so R' stays a proper rotation
    (det is preserved) and can be turned into a quaternion as usual."""
    c = SMAL_TO_UNITY
    c_inv = SMAL_TO_UNITY.T
    return np.einsum("ij,...jl,lm->...im", c, matrices, c_inv)


def main() -> None:
    args = parse_args()
    require_model(args.model)

    with np.load(args.npz) as data:
        anim = prepare_animation(data, args.model)

    rotation_deltas = anim["rotation_deltas"]          # (F, 35, 3, 3)  SMAL space
    world_rotations = anim["world_rotations"]          # (F, 3, 3)      SMAL space
    frame_count, joint_count = rotation_deltas.shape[:2]

    # Bake the SMAL->Unity conversion into every position and rotation.
    rest_joints_u = convert_points(anim["rest_joints"])            # (35, 3)
    world_translations_u = convert_points(anim["world_translations"])  # (F, 3)
    rotation_deltas_u = convert_rotations(rotation_deltas)         # (F, 35, 3, 3)
    world_rotations_u = convert_rotations(world_rotations)         # (F, 3, 3)

    delta_quats = matrices_to_quaternions(rotation_deltas_u)       # (F, 35, 4)
    root_quats = matrices_to_quaternions(world_rotations_u)        # (F, 4)

    payload = {
        "name": args.npz.stem,
        "fps": int(args.fps),
        "frameCount": int(frame_count),
        "jointCount": int(joint_count),
        "boneNamePrefix": "SMAL_joint_",
        # Marker: joints/rotations are already in Unity space (no calibration).
        "space": "unity",
        "parents": anim["parents"].astype(np.int32).tolist(),
        # Unity-space rest joint positions, row-major (F rows of 3).
        "restJoints": rest_joints_u.reshape(-1).astype(np.float64).tolist(),
        # per-frame, per-joint quaternion deltas, flat (F * 35 * 4).
        "deltas": delta_quats.reshape(-1).tolist(),
        # per-frame global body transform (Unity space).
        "rootRot": root_quats.reshape(-1).tolist(),
        "rootPos": world_translations_u.reshape(-1).astype(np.float64).tolist(),
        "rootScale": anim["world_scales"].astype(np.float64).tolist(),
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with open(args.output, "w") as handle:
        json.dump(payload, handle, separators=(",", ":"))

    size_mb = args.output.stat().st_size / 1e6
    print("Exported rig-free Unity SMAL clip (baked into Unity space)")
    print(f"Frames : {frame_count} @ {args.fps} fps")
    print(f"Joints : {joint_count}")
    print(f"Output : {args.output}  ({size_mb:.2f} MB)")


if __name__ == "__main__":
    main()
