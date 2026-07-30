"""Export an InterPet4D dog's neutral shape as a rigged SMAL FBX."""

from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import torch

# chumpy (pulled in by the SMAL model) does ``from numpy import bool, ...`` which
# was removed in NumPy 1.20+. Restore the aliases before anything imports it.
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

from render_interpet_smal import BITE_ROOT, DEFAULT_MODEL, ROOT, require_model


DEFAULT_BLENDER = Path("/Applications/Blender.app/Contents/MacOS/Blender")
BLENDER_SCRIPT = ROOT / "blender_export_smal_fbx.py"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("npz", type=Path, help="InterPet4D smal_npy .npz file")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--blender", type=Path, default=DEFAULT_BLENDER)
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "smal_render/dog02_neutral_rigged.fbx",
    )
    return parser.parse_args()


def prepare_neutral_rig(data: np.lib.npyio.NpzFile, model_path: Path) -> dict:
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

    betas = torch.from_numpy(data["betas"][:1])
    betas_limbs = torch.from_numpy(data["betas_limbs"][:1])
    neutral_pose = torch.eye(3, dtype=betas.dtype).reshape(1, 1, 3, 3)
    neutral_pose = neutral_pose.repeat(1, 35, 1, 1)

    with torch.no_grad():
        vertices, _, _ = model(
            betas,
            betas_limbs,
            pose=neutral_pose,
            keyp_conf="red",
        )

    joints = model.J_transformed[0].cpu().numpy()
    root = joints[0].copy()
    vertices = vertices[0].cpu().numpy() - root
    joints = joints - root
    weights = model.weights.cpu().numpy()

    weight_sums = weights.sum(axis=1)
    if not np.allclose(weight_sums, 1.0, atol=1e-5):
        raise ValueError("SMAL skinning weights do not sum to one")

    return {
        "vertices": vertices.astype(np.float32),
        "faces": model.faces.cpu().numpy().astype(np.int32),
        "joints": joints.astype(np.float32),
        "parents": np.asarray(model.parents, dtype=np.int32),
        "weights": weights.astype(np.float32),
        "betas": data["betas"][0].astype(np.float32),
        "betas_limbs": data["betas_limbs"][0].astype(np.float32),
    }


def main() -> None:
    args = parse_args()
    require_model(args.model)
    if not args.blender.is_file():
        raise FileNotFoundError(f"Blender executable not found: {args.blender}")

    with np.load(args.npz) as data:
        rig_data = prepare_neutral_rig(data, args.model)

    output_path = args.output.resolve()
    with tempfile.TemporaryDirectory(prefix="interpet_smal_fbx_") as temp_dir:
        prepared_path = Path(temp_dir) / "neutral_rig.npz"
        np.savez_compressed(prepared_path, **rig_data)
        subprocess.run(
            [
                str(args.blender),
                "--background",
                "--python",
                str(BLENDER_SCRIPT),
                "--",
                str(prepared_path),
                str(output_path),
            ],
            check=True,
        )

    print("Exported neutral-pose rigged SMAL FBX")
    print(f"Vertices: {len(rig_data['vertices'])}")
    print(f"Faces: {len(rig_data['faces'])}")
    print(f"Joints: {len(rig_data['joints'])}")
    print(f"Output: {output_path}")


if __name__ == "__main__":
    main()
