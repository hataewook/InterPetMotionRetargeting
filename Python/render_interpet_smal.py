"""Render one InterPet4D SMAL frame and export its world-space mesh."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import torch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


ROOT = Path(__file__).resolve().parent
BITE_ROOT = ROOT / "bite_gradio"
DEFAULT_MODEL = (
    BITE_ROOT
    / "data/smal_data/new_dog_models/my_smpl_39dogsnorm_newv3_dog.pkl"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("npz", type=Path, help="InterPet4D smal_npy .npz file")
    parser.add_argument(
        "--frame",
        type=int,
        default=None,
        help="Original frame_idx value (default: first available frame)",
    )
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "smal_render")
    return parser.parse_args()


def require_model(model_path: Path) -> None:
    if not model_path.is_file():
        raise FileNotFoundError(f"SMAL model not found: {model_path}")

    with model_path.open("rb") as model_file:
        header = model_file.read(64)
    if header.startswith(b"version https://git-lfs.github.com/spec"):
        raise RuntimeError(
            f"{model_path} is only a Git LFS pointer. Download the LFS object first."
        )


def select_frame(data: np.lib.npyio.NpzFile, requested_frame: int | None) -> int:
    frame_indices = data["frame_idx"]
    if requested_frame is None:
        return 0

    matches = np.flatnonzero(frame_indices == requested_frame)
    if matches.size == 0:
        raise ValueError(
            f"frame_idx {requested_frame} is absent; available range is "
            f"{frame_indices.min()}..{frame_indices.max()}"
        )
    return int(matches[0])


def create_world_mesh(
    data: np.lib.npyio.NpzFile, frame_position: int, model_path: Path
) -> tuple[np.ndarray, np.ndarray]:
    sys.path.insert(0, str(BITE_ROOT / "src"))
    from smal_pytorch.smal_model.smal_torch_new import SMAL

    model = SMAL(
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

    betas = torch.from_numpy(data["betas"][frame_position : frame_position + 1])
    betas_limbs = torch.from_numpy(
        data["betas_limbs"][frame_position : frame_position + 1]
    )
    pose = torch.from_numpy(
        data["pose_rotmat"][frame_position : frame_position + 1]
    )

    with torch.no_grad():
        local_vertices, _, _ = model(
            betas,
            betas_limbs,
            pose=pose,
            keyp_conf="olive",
        )

    vertices = local_vertices[0].cpu().numpy()
    rotation = data["R_world"][frame_position]
    scale = float(data["s_world"][frame_position])
    translation = data["t_world"][frame_position]
    world_vertices = scale * (rotation @ vertices.T).T + translation
    return world_vertices, model.faces.cpu().numpy()


def export_obj(path: Path, vertices: np.ndarray, faces: np.ndarray) -> None:
    with path.open("w", encoding="utf-8") as obj_file:
        for x, y, z in vertices:
            obj_file.write(f"v {x:.8f} {y:.8f} {z:.8f}\n")
        for i, j, k in faces + 1:
            obj_file.write(f"f {i} {j} {k}\n")


def render_png(
    path: Path,
    vertices: np.ndarray,
    faces: np.ndarray,
    keypoints: np.ndarray,
    weights: np.ndarray,
) -> None:
    triangles = vertices[faces]
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    normal_lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = normals / np.maximum(normal_lengths, 1e-12)
    light = np.array([0.5, -0.4, 1.0])
    light /= np.linalg.norm(light)
    brightness = 0.35 + 0.65 * np.clip(normals @ light, 0.0, 1.0)
    face_colors = brightness[:, None] * np.array([0.35, 0.65, 0.95])

    figure = plt.figure(figsize=(10, 8), dpi=160)
    axes = figure.add_subplot(111, projection="3d")
    axes.add_collection3d(
        Poly3DCollection(triangles, facecolors=face_colors, edgecolors="none")
    )

    visible = weights > 0
    axes.scatter(
        keypoints[visible, 0],
        keypoints[visible, 1],
        keypoints[visible, 2],
        c="crimson",
        s=12,
        depthshade=False,
    )

    lower = vertices.min(axis=0)
    upper = vertices.max(axis=0)
    center = (lower + upper) / 2
    radius = (upper - lower).max() * 0.58
    axes.set_xlim(center[0] - radius, center[0] + radius)
    axes.set_ylim(center[1] - radius, center[1] + radius)
    axes.set_zlim(center[2] - radius, center[2] + radius)
    axes.set_box_aspect((1, 1, 1))
    axes.set_xlabel("world X (m)")
    axes.set_ylabel("world Y (m)")
    axes.set_zlabel("world Z (m)")
    axes.view_init(elev=20, azim=-60)
    figure.tight_layout()
    figure.savefig(path, bbox_inches="tight")
    plt.close(figure)


def main() -> None:
    args = parse_args()
    require_model(args.model)

    with np.load(args.npz) as data:
        frame_position = select_frame(data, args.frame)
        frame_index = int(data["frame_idx"][frame_position])
        vertices, faces = create_world_mesh(data, frame_position, args.model)
        keypoints = data["kp_world"][frame_position]
        weights = data["kp_weight"][frame_position]

    args.output_dir.mkdir(parents=True, exist_ok=True)
    stem = f"{args.npz.stem}_frame{frame_index:05d}"
    obj_path = args.output_dir / f"{stem}.obj"
    png_path = args.output_dir / f"{stem}.png"
    export_obj(obj_path, vertices, faces)
    render_png(png_path, vertices, faces, keypoints, weights)
    print(f"Rendered frame_idx={frame_index}")
    print(f"Mesh: {obj_path}")
    print(f"Image: {png_path}")


if __name__ == "__main__":
    main()
