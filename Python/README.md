# Python — InterPet4D → Unity conversion pipeline

A collection of scripts that convert InterPet4D dog motion (`.npy` / `.npz`) into the
formats Unity consumes at runtime. There are two main branches:

1. **SMAL clip (`.npz` → JSON / FBX)** — drive the SMAL rig with per-frame quaternion deltas.
2. **Keypoint pose (`.npy` → JSON)** — export the 20 keypoint world coordinates as-is for FK aiming in Unity.

---

## Environment

- conda env: **`smal`** (`conda activate smal`)
- SMAL model: `bite_gradio/data/smal_data/new_dog_models/my_smpl_39dogsnorm_newv3_dog.pkl`
  (must be the real file, not a Git LFS pointer — `require_model()` checks this)
- SMAL Python implementation: `bite_gradio/src/smal_pytorch/...` (scripts add it to `sys.path` at runtime)
- FBX export needs Blender: default `/Applications/Blender.app/Contents/MacOS/Blender`

---

## Directory

```
Python/
├── *.py                 # active scripts (table below)
├── bite_gradio/         # SMAL model (.pkl) + smal_pytorch code  ← required dependency
├── datasets/            # input data
│   └── interpet4d/
│       ├── smal_npy/    #   *.npz  (SMAL pose_rotmat, betas, R/t/s_world, kp_world ...)
│       └── pet_npy/     #   *.npy  ((F, 20, 4) = keypoint xyz + confidence)
├── smal_render/         # output artifacts such as FBX
├── deprecated/          # past scripts / data (SMALify, smpl_webuser, target_*, etc.)
└── README.md
```

---

## Active scripts

| Script | Role | Input → Output |
|---|---|---|
| [`export_smal_unity_clip.py`](export_smal_unity_clip.py) | **SMAL motion → Unity runtime clip.** Exports per-frame joint rotation deltas (quaternions) relative to the rest pose plus the global root transform, as JSON. Consumed by Unity `SmalMotionPlayer`. | `smal_npy/*.npz` → `Assets/SmalClips/<name>.json` |
| [`export_pose_positions.py`](export_pose_positions.py) | **20 keypoint world coordinates → Unity JSON.** Exports the raw coordinates with no conversion; axis remap / scale / root alignment are handled by Unity `DogPoseDriver`. | `pet_npy/*.npy` (F,20,4) → `Assets/KeypointPoses/<name>.json` |
| [`export_interpet_smal_rigged_fbx.py`](export_interpet_smal_rigged_fbx.py) | **Neutral-shape rigged SMAL FBX.** Exports a skinned mesh + armature with only the betas shape applied, via Blender. | `smal_npy/*.npz` → `smal_render/*_neutral_rigged.fbx` |
| [`export_interpet_smal_animated_fbx.py`](export_interpet_smal_animated_fbx.py) | **Bakes the whole sequence into an animated rigged FBX.** `prepare_animation()` (rotation deltas + rest joints) lives here and is reused by `export_smal_unity_clip`. | `smal_npy/*.npz` → `smal_render/*_animated_rigged.fbx` |
| [`render_interpet_smal.py`](render_interpet_smal.py) | **Shared base.** `ROOT`, `BITE_ROOT`, `DEFAULT_MODEL`, `require_model()`, SMAL world-mesh utilities. Imported by the scripts above. Run standalone to render one frame / export a mesh. | `smal_npy/*.npz` → mesh/render |
| [`blender_export_smal_fbx.py`](blender_export_smal_fbx.py) | Blender-internal helper — builds/skins a weighted SMAL armature + mesh and exports FBX. Invoked by the `rigged_fbx` script via `blender --python`. | (subprocess) |
| [`blender_export_smal_animated_fbx.py`](blender_export_smal_animated_fbx.py) | Blender-internal helper — bakes motion onto the rig from the helper above. Invoked by the `animated_fbx` script. | (subprocess) |

### Dependencies

```
export_smal_unity_clip ─┐
                        ├─> export_interpet_smal_animated_fbx ─> export_interpet_smal_rigged_fbx ─┐
                        └─> render_interpet_smal <───────────────────────────────────────────────┘
                                                                (blender_export_smal_animated_fbx
                                                                 └─ imports blender_export_smal_fbx)
export_pose_positions   (standalone — uses only numpy/json)
```

> `blender_export_smal_*.py` are invoked as **subprocess path strings**, not via `import`,
> so the dependency is invisible to import tracing. To use the FBX scripts, these two
> helpers must be at the top level of `Python/`.

---

## Usage examples

```bash
conda activate smal
cd Python

# 1) SMAL motion → Unity clip JSON
python export_smal_unity_clip.py \
    datasets/interpet4d/smal_npy/interpet_dog09_p19_take04_ego_002.npz \
    --output ../Assets/SmalClips/interpet_dog09_p19_take04_ego_002.json

# 2) Keypoints → Unity pose JSON
python export_pose_positions.py \
    datasets/interpet4d/pet_npy/interpet_dog09_p20_take01_ego_001.npy \
    ../Assets/KeypointPoses/DogPose_dog09_p20_take01.json

# 3) Neutral rigged FBX (needs Blender)
python export_interpet_smal_rigged_fbx.py \
    datasets/interpet4d/smal_npy/interpet_dog09_p19_take04_ego_002.npz \
    --output smal_render/interpet_dog09_neutral_rigged.fbx

# 4) Animated rigged FBX (needs Blender)
python export_interpet_smal_animated_fbx.py \
    datasets/interpet4d/smal_npy/interpet_dog02_p04_take01_ego_001.npz \
    --output smal_render/interpet_dog02_animated_rigged.fbx
```

---

## Output JSON schema

**SMAL clip** (`export_smal_unity_clip.py`, consumed by Unity `SmalMotionPlayer`)

```jsonc
{
  "name": "...", "fps": 60, "frameCount": N, "jointCount": 35,
  "boneNamePrefix": "SMAL_joint_",
  "parents": [ ...35 ],
  "restJoints": [ ...35*3 ],        // rest joint positions in armature space (row-major)
  "deltas":   [ ...N*35*4 ],        // per-frame, per-joint rotation delta quaternion (x,y,z,w)
  "rootRot":  [ ...N*4 ],           // R_world quaternion
  "rootPos":  [ ...N*3 ],           // t_world
  "rootScale":[ ...N ]              // s_world
}
```

**Keypoint pose** (`export_pose_positions.py`, consumed by Unity `DogPoseDriver`)

```jsonc
{
  "clipName": "...", "frameCount": N, "jointCount": 20, "fps": 30.0,
  "jointNames": [ ...20 ],          // L_Eye, R_Eye, ... , L_B_Paw, R_B_Paw
  "parents":    [ ...20 ],
  "frames": [ { "p": [ ...20*3 ],   // world coordinates (xyz, raw)
                "s": [ ...20 ] } ]  // confidence (4th channel of the npy)
}
```
