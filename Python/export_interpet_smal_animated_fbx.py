"""Bake a complete InterPet4D SMAL sequence to a separate rigged FBX."""

from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import torch

from export_interpet_smal_rigged_fbx import DEFAULT_BLENDER
from render_interpet_smal import BITE_ROOT, DEFAULT_MODEL, ROOT, require_model


BLENDER_SCRIPT = ROOT / "blender_export_smal_animated_fbx.py"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("npz", type=Path, help="InterPet4D smal_npy .npz file")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--blender", type=Path, default=DEFAULT_BLENDER)
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "smal_render/interpet_dog02_animated_rigged.fbx",
    )
    return parser.parse_args()


def load_smal(model_path: Path):
    sys.path.insert(0, str(BITE_ROOT / "src"))
    from smal_pytorch.smal_model.smal_torch_new import SMAL

    return SMAL(
        pkl_path=str(model_path),
        logscale_part_list=[
            "legs_l",
            "legs_f",
            "tail_l",
            "tail_f",
            "ears_y",
            "ears_l",
            "head_l",
        ],
    ).eval()


def calculate_shaped_joints(model, betas: torch.Tensor) -> torch.Tensor:
    shaped_vertices = model.v_template + torch.reshape(
        torch.matmul(betas, model.shapedirs[: betas.shape[1]]),
        [-1, model.size[0], model.size[1]],
    )
    return torch.stack(
        [
            torch.matmul(shaped_vertices[:, :, axis], model.J_regressor)
            for axis in range(3)
        ],
        dim=2,
    )


def calculate_global_transforms(
    rotations: torch.Tensor,
    shaped_joints: torch.Tensor,
    parents: np.ndarray,
    scale_factors: torch.Tensor,
) -> torch.Tensor:
    batch_size = rotations.shape[0]
    joints = shaped_joints.repeat(batch_size, 1, 1)
    scales = scale_factors.repeat(batch_size, 1, 1, 1)
    transforms = []

    root = torch.eye(4, dtype=rotations.dtype).repeat(batch_size, 1, 1)
    root[:, :3, :3] = rotations[:, 0]
    root[:, :3, 3] = joints[:, 0]
    transforms.append(root)

    for joint_index in range(1, len(parents)):
        parent = int(parents[joint_index])
        local = torch.eye(4, dtype=rotations.dtype).repeat(batch_size, 1, 1)
        local[:, :3, :3] = (
            torch.linalg.inv(scales[:, parent])
            @ rotations[:, joint_index]
            @ scales[:, joint_index]
        )
        local[:, :3, 3] = joints[:, joint_index] - joints[:, parent]
        transforms.append(transforms[parent] @ local)

    return torch.stack(transforms, dim=1)


def closest_rotations(matrices: np.ndarray) -> np.ndarray:
    u, _, vh = np.linalg.svd(matrices)
    rotations = u @ vh
    reflected = np.linalg.det(rotations) < 0
    if reflected.any():
        u[reflected, :, -1] *= -1
        rotations[reflected] = u[reflected] @ vh[reflected]
    return rotations


def prepare_animation(data: np.lib.npyio.NpzFile, model_path: Path) -> dict:
    model = load_smal(model_path)
    parents = np.asarray(model.parents, dtype=np.int32)
    betas = torch.from_numpy(data["betas"][:1])
    betas_limbs = torch.from_numpy(data["betas_limbs"][:1])
    shaped_joints = calculate_shaped_joints(model, betas)
    scale_factors = torch.diag_embed(
        torch.exp(betas_limbs @ model.betas_scale_mask).reshape(1, 35, 3)
    )

    neutral_pose = torch.eye(3, dtype=betas.dtype).reshape(1, 1, 3, 3)
    neutral_pose = neutral_pose.repeat(1, 35, 1, 1)
    with torch.no_grad():
        neutral_vertices, _, _ = model(
            betas,
            betas_limbs,
            pose=neutral_pose,
            keyp_conf="red",
        )
        neutral_transforms = calculate_global_transforms(
            neutral_pose, shaped_joints, parents, scale_factors
        )

    all_transforms = []
    poses = torch.from_numpy(data["pose_rotmat"])
    with torch.no_grad():
        for start in range(0, len(poses), 64):
            all_transforms.append(
                calculate_global_transforms(
                    poses[start : start + 64], shaped_joints, parents, scale_factors
                )
            )
    transforms = torch.cat(all_transforms).cpu().numpy()
    neutral_transforms = neutral_transforms[0].cpu().numpy()

    inverse_neutral_linear = np.linalg.inv(neutral_transforms[:, :3, :3])
    linear_deltas = transforms[:, :, :3, :3] @ inverse_neutral_linear[None]
    rotation_deltas = closest_rotations(linear_deltas.reshape(-1, 3, 3)).reshape(
        len(transforms), 35, 3, 3
    )

    rest_joints = neutral_transforms[:, :3, 3]
    root = rest_joints[0].copy()
    local_joints = transforms[:, :, :3, 3] - root
    neutral_vertices = neutral_vertices[0].cpu().numpy() - root
    rest_joints = rest_joints - root
    weights = model.weights.cpu().numpy()

    if not np.allclose(weights.sum(axis=1), 1.0, atol=1e-5):
        raise ValueError("SMAL skinning weights do not sum to one")

    return {
        "vertices": neutral_vertices.astype(np.float32),
        "faces": model.faces.cpu().numpy().astype(np.int32),
        "rest_joints": rest_joints.astype(np.float32),
        "parents": parents,
        "weights": weights.astype(np.float32),
        "local_joints": local_joints.astype(np.float32),
        "rotation_deltas": rotation_deltas.astype(np.float32),
        "world_rotations": data["R_world"].astype(np.float32),
        "world_translations": data["t_world"].astype(np.float32),
        "world_scales": data["s_world"].astype(np.float32),
        "frame_indices": data["frame_idx"].astype(np.int32),
        "betas": data["betas"][0].astype(np.float32),
        "betas_limbs": data["betas_limbs"][0].astype(np.float32),
    }


def main() -> None:
    args = parse_args()
    require_model(args.model)
    if not args.blender.is_file():
        raise FileNotFoundError(f"Blender executable not found: {args.blender}")

    with np.load(args.npz) as data:
        animation_data = prepare_animation(data, args.model)

    output_path = args.output.resolve()
    previous_mtime = output_path.stat().st_mtime_ns if output_path.exists() else None
    with tempfile.TemporaryDirectory(prefix="interpet_smal_animation_") as temp_dir:
        prepared_path = Path(temp_dir) / "animated_rig.npz"
        np.savez_compressed(prepared_path, **animation_data)
        subprocess.run(
            [
                str(args.blender),
                "--background",
                "--factory-startup",
                "--python",
                str(BLENDER_SCRIPT),
                "--",
                str(prepared_path),
                str(output_path),
            ],
            check=True,
        )

    if not output_path.is_file():
        raise RuntimeError(f"Blender did not create the FBX: {output_path}")
    if previous_mtime is not None and output_path.stat().st_mtime_ns == previous_mtime:
        raise RuntimeError(f"Blender did not update the FBX: {output_path}")

    print("Exported animated rigged SMAL FBX")
    print(f"Samples: {len(animation_data['frame_indices'])}")
    print(
        f"Timeline: {animation_data['frame_indices'][0]}.."
        f"{animation_data['frame_indices'][-1]} at 60 fps"
    )
    print(f"Output: {output_path}")


if __name__ == "__main__":
    main()
