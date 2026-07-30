"""
Export InterPet4D 20-keypoint world positions to a Unity-friendly JSON that
DogPoseDriver.cs consumes for world-space aim FK.

Positions are exported RAW (no coordinate conversion). The npy->Unity axis
remap, scale and root alignment are done on the Unity side so they can be
tuned live in the Inspector without re-exporting.

Schema (JsonUtility-compatible: object with an array of objects, float[] fields):
{
  "clipName": "...",
  "frameCount": 326,
  "jointCount": 20,
  "fps": 30.0,
  "jointNames": [...20...],
  "parents":    [...20...],
  "frames": [ { "p": [x0,y0,z0, ...20*3], "s": [s0..s19] }, ... ]
}
"""

import json
import os
import sys

import numpy as np

JOINT_NAMES = [
    "L_Eye", "R_Eye", "L_EarBase", "R_EarBase", "Nose",
    "Throat", "TailBase", "Withers", "L_F_Knee", "R_F_Knee",
    "L_B_Knee", "R_B_Knee", "L_F_Elbow", "R_F_Elbow",
    "L_B_Elbow", "R_B_Elbow", "L_F_Paw", "R_F_Paw",
    "L_B_Paw", "R_B_Paw",
]
PARENTS = [-1, 0, 0, 1, 2, 0, 7, 5, 5, 5, 6, 6, 8, 9, 10, 11, 12, 13, 14, 15]
NUM_JOINTS = len(JOINT_NAMES)


def export(src, dst, fps=30.0):
    data = np.load(src)
    assert data.ndim == 3 and data.shape[1:] == (NUM_JOINTS, 4), \
        f"expected (F, {NUM_JOINTS}, 4), got {data.shape}"
    F = data.shape[0]
    pos = data[..., :3].astype(np.float32)
    conf = data[..., 3].astype(np.float32)

    frames = []
    for f in range(F):
        frames.append({
            "p": [float(v) for v in pos[f].reshape(-1)],   # 60 floats
            "s": [float(v) for v in conf[f]],              # 20 floats
        })

    payload = {
        "clipName": os.path.splitext(os.path.basename(src))[0],
        "frameCount": F,
        "jointCount": NUM_JOINTS,
        "fps": float(fps),
        "jointNames": JOINT_NAMES,
        "parents": PARENTS,
        "frames": frames,
    }

    os.makedirs(os.path.dirname(dst), exist_ok=True)
    with open(dst, "w") as fp:
        json.dump(payload, fp)
    print(f"wrote {dst}  frames={F}  fps={fps}")


if __name__ == "__main__":
    src = sys.argv[1] if len(sys.argv) > 1 \
        else "datasets/interpet4d/pet_npy/interpet_dog01_p01_take01_ego_001.npy"
    dst = sys.argv[2] if len(sys.argv) > 2 \
        else "../KeypointPoses/DogPose_dog01_p01_take01.json"
    fps = float(sys.argv[3]) if len(sys.argv) > 3 else 30.0
    export(src, dst, fps)
