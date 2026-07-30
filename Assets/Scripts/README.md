# Assets/Scripts — Runtime Motion Drivers

C# scripts that play back / retarget InterPet4D motion JSON (exported from `Python/`)
onto dog rigs **at runtime** in Unity. No baked animation clips are used; the bones are
driven directly every frame. Everything lives in the `PetDemo` namespace.

The pipeline mirrors the Python side and comes in two branches:

- **A. Keypoint pose** — 20-keypoint JSON from `export_pose_positions.py` → `DogPoseDriver`
- **B. SMAL clip** — SMAL rotation-delta JSON from `export_smal_unity_clip.py` → `SmalMotionPlayer` → retarget

---

## A. Keypoint pose pipeline

Reads 20-keypoint world coordinates from `.npy`
(→ [`Assets/KeypointPoses/*.json`](../KeypointPoses/)) and drives a dog rig.

| Script | Role |
|---|---|
| [`DogPoseClip.cs`](DogPoseClip.cs) | Data model. One frame = 20 joint world positions (flat `xyz*20`) + confidence. Layout matches the `export_pose_positions.py` output 1:1 (`DogPoseFrame`, `DogPoseClip`). |
| [`DogJoints.cs`](DogJoints.cs) | InterPet4D MMPose keypoint index `enum DogKeypoint` (20 of them), rig presets (`DogRigPreset`), joint-mapping helper (`DogJoints`). |
| [`DogPoseDriver.cs`](DogPoseDriver.cs) | **Main driver.** Reads a clip, solves the legs with IK/FK (`LegMode`), **aims the head** (= rotates the `Throat` bone toward the mapped bone of `headAimKeypoint` (eye/nose), `aimHead` option), and exposes capture-axis→Unity-axis remap / scale / root alignment in the Inspector. Holds several clips and switches at runtime via `Clip Index`. |
| [`NormalizedDogPoseDriver.cs`](NormalizedDogPoseDriver.cs) | Derives from `DogPoseDriver`. Preserves the target rig's scale and maps each leg relative to its root, normalized to the target leg's bind length. |

## B. SMAL clip pipeline

Drives the SMAL neutral rig with rotation deltas, then retargets that motion onto a real
target rig (Shepherd, etc.).

| Script | Role |
|---|---|
| [`SmalMotionPlayer.cs`](SmalMotionPlayer.cs) | **Drives the SMAL source rig.** Poses bones `SMAL_joint_00`..`SMAL_joint_34` every frame from the per-frame world rotation deltas + global body transform in the `export_smal_unity_clip.py` clip. No baking. |
| [`SmalRetargeter.cs`](SmalRetargeter.cs) | **SMAL → target rig retarget.** Legs use IK/FK (`RetargetMode`) + an **optional toe FK** (hybrid: plant the paw with IK, add the toe with FK on top); the **spine/neck/tail use per-segment FK** (reproduces the whole back/tail curve); the **jaw/ears (32/33/34) copy local rotation** (with axis remap); an optional head goal is solved with CCD; global motion is carried over. Spine FK moves the shoulders, so it is solved before the legs and the torso frame is re-measured afterward. Joint mapping via `SmalRetargetMap`. |
| [`SmalRetargetMap.cs`](SmalRetargetMap.cs) | Retarget mapping definitions. `RetargetLeg` (upper→lower→tip two-bone IK chain + pole/root SMAL joints + **optional toe bone/joint**), `RetargetChain` (CCD), **`RetargetFkChain`** (shared per-segment FK chain for spine/neck/tail), **`RetargetCopyRotation`** (jaw/ear local-rotation copy + `Axis` remap), and the static map `SmalRetargetMap` (default spine 1→6, neck 15→16, tail 25→31, jaw 32, ears 33/34). |
| [`SmalIkTargetDriver.cs`](SmalIkTargetDriver.cs) | **Drives Animation Rigging IK targets.** Moves the Shepherd's IK targets to the joint world positions of the live SMAL rig (posed by `SmalMotionPlayer`) → the Shepherd's TwoBoneIKConstraints solve legs/nose/tail/trunk to them, and root motion is carried over. Requires an active `RigBuilder`. |

## Shared IK / smoothing utilities (used by both branches)

| Script | Role |
|---|---|
| [`TwoBoneIK.cs`](TwoBoneIK.cs) | Analytic two-bone IK. Rotates `upper`/`lower` (parent→child) so the tip reaches the target, with `pole` fixing the bend plane. **Rotation only** — bone lengths are preserved (proportions kept). An optional `softZone` argument **eases only the region just before full extension** with an exponential saturation, softening the "knee snap" as the leg straightens (0 = the old hard clamp). |
| [`ChainIK.cs`](ChainIK.cs) | CCD IK for arbitrary-length chains. Rotates each bone base→tip to bring the effector to the target, with an optional pole to fix the bend plane. Rotation only — length preserved. |
| [`OneEuroFilterVector3.cs`](OneEuroFilterVector3.cs) | One-Euro low-pass filter for Vector3. Adapts its cutoff — **strong smoothing when slow, no lag when fast** — to kill capture jitter. Applied by both drivers to keypoint / source-joint positions. |
| [`FootPlanter.cs`](FootPlanter.cs) | Foot-contact lock (anti-skate). Detects stance from the speed of a **detect point** (the body-independent raw source foot) and freezes the **value** (IK goal / tip) during stance to remove foot sliding. Plant/release is eased over `attack` seconds. Detect and value are separate, so it also works for a body-relative (normalized) goal. |
| [`RootAxisMask.cs`](RootAxisMask.cs) | Shared helper that restricts a root-align translation/rotation to **selected world axes only** (`[Flags] RootAxis`). Rotation is masked by zeroing unselected components of its Euler decomposition (single-axis exact, combinations approximate). Used by both drivers for root/global alignment. |

`DogPoseClip` / `DogJoints` / `SmalRetargetMap` are data/mapping definitions, not components, so they are never attached in the Inspector.

> **Mode-based field hiding in the Inspector**: custom editors ([`Editor/DogPoseDriverEditor.cs`](../Editor/DogPoseDriverEditor.cs), [`Editor/SmalRetargeterEditor.cs`](../Editor/SmalRetargeterEditor.cs)) hide IK-only fields (pole weights, IK roll, `Max Leg Roll Degrees`) when `Leg Mode` is **FK**, and hide the axis-mask fields when `Align/Apply Global Motion` is **off**. Hidden values are not cleared — only not shown.

---

## Inspector setup (setup guide)

Components are meant to be attached to the rig **root** (the object whose children are the
skeleton). Below is what to assign on each component and the order to operate in.

### `SmalMotionPlayer` — attach to the SMAL source rig

Attach to the SMAL rig (the object whose children are `SMAL_joint_00..34`).

| Group | Field | What to assign / do |
|---|---|---|
| **Clip** | `Clip Json` | The SMAL clip JSON produced by `Python/export_smal_unity_clip.py` (`Assets/SmalClips/*.json`). **Required.** |
| **Rig** | `Model Root` | Parent of the SMAL bones. Empty = the transform this component is on. |
| **Playback** | `Fps Override` | 0 = the clip's own fps. Enter a value only to force a playback speed. |
| | `Loop` | Loop playback on/off. |
| | `Apply Global Motion` | Apply the captured whole-body translation/rotation/scale. Off = stay in place, articulation only. |
| **Debug** | `Log Calibration` | Log the rest-joint fit error (m) at startup. A large value signals a rig/clip mismatch. |

### `SmalRetargeter` — attach to the target rig (Shepherd, etc.) root

Transfers SMAL motion onto the target rig. Handles **legs (IK/FK) + trunk (CCD) + tail (FK)** in one component.

| Group | Field | What to assign / do |
|---|---|---|
| **Source** | `Smal Source` | Root of the SMAL rig driven by `SmalMotionPlayer`. **Required.** |
| | `Bone Name Prefix` | Source bone prefix. Default `SMAL_joint_` (two-digit index appended). |
| **Rig** | `Target Root` | Root of the target rig to drive. Empty = this transform. |
| **Torso frame bones** | `Left/Right Shoulder Bone`, `Left/Right Hip Bone` | The target rig's 4 shoulder/hip bones. They build the torso frame (forward/lateral/up), so **all required.** |
| **Legs** | `Leg Mode` | `IK` = plant the feet at the SMAL foot positions (recommended), `FK` = transfer leg directions only (feet may float). |
| | `Max Leg Roll Degrees` | Leg twist allowed in IK mode (0 = fully locked). **Hidden in FK.** |
| | `Ik Soft Zone` | Eases only the region just before full extension (default 3%) to soften the knee snap (0 = off). Used by both IK and the FK stance-plant correction. |
| | `Legs` | Array of 4 legs. In each element drag `Upper/Lower/Tip Bone` (a parent→child chain). `Root/Target/Pole Joint` (SMAL joint numbers) come pre-filled (front 7/9/8, 11/13/12; hind 17/19/18, 21/23/22). **`Toe Bone` (optional)**: assign the toe bone (child of tip) to plant the paw with IK and then drive only the toe with FK. `Toe Joint` = 0 auto-fills to `targetJoint+1` (front 10/14, hind 20/24). Leave empty to skip the toe FK. |
| **Trunk / head (CCD, optional)** | `Chains` | CCD chains for a single end goal (e.g. head aim). Each element: `Base Bone`→`Effector Bone` (a contiguous parent→child chain) and SMAL `Root/End Joint`. **The back is EITHER this or the spine FK below** — do not put both on the same bones. |
| | `Ik Iterations` | CCD iteration count (default 10). Increase if it does not reach. |
| **Spine / neck / tail (per-segment FK)** | `Fk Chains` | Array of per-segment FK chains for spine/neck/tail. Name each element `spine`/`neck`/`tail` and **drag its bones into `Bones` base→tip**; then `Joints` (spine 1..6, neck 15..16, tail 25..31) auto-fills (can also be set explicitly). Reproduces the whole back curve, absorbs length differences, no twist. Remove chains you don't use. |
| **Copy rotation (jaw / ears)** | `Copy Rotations` | Array of jaw/ears (default Jaw 32, L_Ear 33, R_Ear 34). Assign a **`Target Bone`** (the target jaw/ear bone) to copy **only the SMAL joint's local rotation (its change from neutral)**. If the two rigs' axes differ, use **`Map X/Y/Z`** to say which target axis (±X/Y/Z) each SMAL local x/y/z rotation goes to (default X+/Y+/Z+ = as-is). **`Weight`** (0–2): angle multiplier — 1 = 1:1, `<1` suppresses, 0 = frozen, `>1` exaggerates. Adjustable live in Play. Leave Target Bone empty to skip. |
| **Global motion** | `Apply Global Motion` | On = the target body follows the SMAL body's translation/rotation; off = stay in place, articulation only. |
| | `Global Rotation Axes` / `Global Translation Axes` | **Select the world axes** the body overlay may act on (shown only when `Apply Global Motion` is on). The overlay matches the full torso frame (forward + up), so a roll/pitch can flip the whole body → clear Z/X and keep Y for turning (yaw) only. Single axis exact, combinations approximate. |
| **Debug** | `Log Calibration` / `Draw Gizmos` / `Gizmo Radius` | Calibration log; goal gizmos (green = goal, yellow = pole). |

> Leg / CCD / FK chain bones must be a **contiguous parent→child chain**; otherwise setup throws an exception to tell you.

### (not used) `SmalIkTargetDriver` — attach to the Animation Rigging Shepherd root

Use this to retarget via **Animation Rigging** instead of `SmalRetargeter` (pick one).
The target rig must already have a `RigBuilder` + `TwoBoneIKConstraint`s set up.

| Group | Field | What to assign / do |
|---|---|---|
| **Source** | `Smal Source` | SMAL rig root. **Required.** `Bone Name Prefix` is the same as above. |
| **Rig** | `Target Root` | Shepherd root that receives root motion. Empty = this transform. |
| **Target mapping** | `Bindings` | One element per IK target: `Target` (the IK constraint's Target transform) + `Joint` (the SMAL joint number to follow). Map the leg/nose/tail/trunk targets individually. **At least 1 required.** |
| | `Preserve Initial Offset` | Keep the startup gap instead of snapping the target onto the joint (when the two rigs don't match at bind). |
| **Root motion** | `Apply Root Motion` / `Root Follow Position` / `Root Follow Rotation` | Make the Shepherd root follow the SMAL body's translation/rotation. |
| **Debug** | `Draw Gizmos` / `Gizmo Radius` | Target-position gizmos (magenta). |

### `DogPoseDriver` / `NormalizedDogPoseDriver` — attach to the dog rig root

For the keypoint pose pipeline. `NormalizedDogPoseDriver` has the same fields; it is a
derivative that normalizes the legs to be root-relative and bind-length.

| Group | Field | What to assign / do |
|---|---|---|
| **Clips** | `Clips` | Array of 20-keypoint pose JSON (`KeypointPoses/*.json`). Switch at runtime with `Clip Index`. |
| **Rig** | `Dog Root` | Target skeleton root. Empty = this transform. |
| | `Mapping Preset` | Rig preset (`DogRejoint`, etc.) — auto-determines the keypoint↔bone mapping. |
| | `Joint Bones` | Assign joint bones manually when the preset doesn't fit (manual override). |
| **Head** | `Aim Head` / `Head Aim Keypoint` | Rotate the `Throat` bone toward the eye/nose keypoint to aim the head. |
| **Leg solving** | `Leg Mode` | `IK` (follows paw positions) / `FK` (transfers directions). |
| **Solver weights** | each `... Weight` (0–1) | body position/rotation, head aim, per-leg / pole weights. Turn parts down partially. **Pole weights are IK-only, so hidden in FK.** |
| **IK leg roll** | `Limit Ik Leg Roll` / `Max Ik Leg Roll Degrees` | Limit leg twist (default 15°). **Hidden in FK.** |
| **IK soft limit** | `Ik Soft Zone` | Eases only the region just before full extension (default 3%) to soften the knee snap (0 = off). Used by both IK and the FK stance-plant correction. |
| **Smoothing** | `Smooth Keypoints` / `Smoothing Min Cutoff` / `Smoothing Beta` | One-Euro filter on the captured keypoints. `Min Cutoff`↓ = smoother but more lag; `Beta`↑ = follows fast motion better. |
| **Foot contact lock** | `Lock Planted Feet` / `Foot Lock Speed` / `Foot Unlock Speed` / `Foot Lock Attack` | Freeze a planted foot in capture world space to remove sliding. `Lock/Unlock Speed` (m/s) decide stance (hysteresis), `Attack` (s) eases. In FK it plants the foot with a light IK correction only during stance. |
| **Capture → Unity axes** | `Swap YZ` / `Axis Sign` / `Capture Origin` | Capture (Z-up) → Unity (Y-up) axis conversion. Flip a sign with `Axis Sign` if left/right or front/back come out wrong. Place with `Capture Origin`. |
| **Scale** | `Auto Scale Dog To Capture` / `Scale Multiplier` | Auto-fit the dog to the capture + a multiplier for fine tuning. |
| **Playback** | `Fps Override` / `Loop` | Same as `SmalMotionPlayer` above. |
| **Root** | `Align Root To Trunk` | Align the body along `TailBase→Throat`. |
| | `Align Rotation Axes` | **Select the world axes** the trunk-align rotation acts on (shown only when `Align Root To Trunk` is on). E.g. clear Z to stop a body-flipping roll; keep Y only for yaw. |
| | `Align Translation Axes` | **Select the world axes** the root follows the captured body centre on. E.g. clear Y to stay at ground height. |
| **Confidence** | `Hold Last Valid` / `Confidence Threshold` | Drop low-confidence frames and hold the last valid pose. |
| **Debug gizmos** | `Draw Keypoint Gizmos` / `Draw Joint Labels` / `Gizmo Joint Radius` | Play-mode keypoint gizmos/labels. |

### Typical setup order (B pipeline)

1. Place the SMAL rig in the scene, attach `SmalMotionPlayer` → assign **Clip Json**.
2. Place the target rig (Shepherd, etc.), attach `SmalRetargeter` (or `SmalIkTargetDriver`).
3. Drag the SMAL rig from step 1 into **Smal Source**.
4. Assign the 4 shoulder/hip bones → the leg/spine/neck/tail bone chains (back = spine FK OR trunk CCD, pick one) → jaw/ear Target Bones if needed.
5. Play. Verify the mapping with the gizmos / `Log Calibration`, then adjust axes, scale, and weights.

---

## Data flow

```
[A] pet_npy/*.npy ─(export_pose_positions.py)─► KeypointPoses/*.json
        └─► DogPoseClip ─► DogPoseDriver / NormalizedDogPoseDriver ─► cartoon dog rig
                                  └─ TwoBoneIK / ChainIK

[B] smal_npy/*.npz ─(export_smal_unity_clip.py)─► SmalClips/*.json
        └─► SmalMotionPlayer ─► SMAL source rig (SMAL_joint_00..34)
                 ├─► SmalRetargeter (+SmalRetargetMap) ──► target rig (Shepherd)
                 └─► SmalIkTargetDriver ──► Animation Rigging IK targets ─► Shepherd
                          └─ TwoBoneIK / ChainIK
```

## Notes

- **Head aim**: only rotates the `Throat` bone toward the "neck→eye (or nose) keypoint"
  direction to aim the head (not positional IK). `aimHead` toggles it; `headAimKeypoint`
  picks the target keypoint.
- **Bone length preserved**: every IK (`TwoBoneIK`/`ChainIK`) applies rotation only — it
  never breaks the target rig's proportions.
- **Frame interpolation (no stepping)**: both `DogPoseDriver` and `SmalMotionPlayer`
  interpolate between adjacent frames instead of snapping to integer frames (keypoints =
  `Lerp`, SMAL rotation deltas / root rotation = `Slerp`, root position/scale = `Lerp`). A
  low-fps clip played at high render fps shows no stepping. Automatic, no Inspector field.
- **Source smoothing (One-Euro)**: each driver filters its keypoint / SMAL joint positions
  with `OneEuroFilterVector3` to remove jitter (strong when slow, no lag when fast).
  `SmalRetargeter` caches `SmalPos` once per LateUpdate so each joint's filter advances
  exactly once.
- **Foot contact lock (anti-skate)**: `FootPlanter` detects stance and freezes the foot in
  world space during contact. IK mode freezes the IK goal; FK mode is a hybrid that adds a
  light IK correction only during stance. **Detection uses the raw source foot** (a
  body-independent space) while the frozen value is the goal/tip, so it also works for a
  normalized (body-relative) goal.
- **Root-align axis mask**: `RootAxisMask` restricts `DogPoseDriver`'s trunk align and
  `SmalRetargeter`'s global overlay to **selected world axes only**. This is aimed
  especially at the SMAL overlay, which matches the full torso frame so a roll/pitch can
  flip the body — clear Z (and X) and keep Y (yaw). Single-axis masks are exact; multi-axis
  combinations are approximate (Euler-order dependent).
- **Straight-leg twist (known limitation)**: when the upper/lower are nearly collinear (as
  on a standing front leg), the pole / bend plane becomes ill-conditioned and tiny source
  noise can be amplified into upper-bone roll. Lowering `Max Leg Roll Degrees` on
  `SmalRetargeter` (e.g. 0) removes the twist, but with the trade-off that the upper bone's
  legitimate knee (pole) direction is also locked to bind. The real fix is to **stabilize
  the pole itself** rather than shaving the resulting roll (not yet implemented).
- **Per-segment FK chains (spine/neck/tail)**: `SmalRetargeter` transfers the per-segment
  directions of the SMAL chains (spine 1→6, neck 15→16, tail 25→31) onto the target bones
  (shared `RetargetFkChain` logic). Rotation only, so length differences are absorbed and
  no twist is introduced (roll kept at bind); it reproduces the **whole curve**, not just
  the endpoint. When bone counts differ, SMAL segments are distributed proportionally (1:1
  when equal). In the Inspector, name a chain spine/neck/tail and just drag the bone chain —
  the SMAL joints auto-fill.
- **Spine FK before the legs**: because the spine moves the shoulders (torso-frame bones),
  `LateUpdate` first solves the FK chains that "move the torso" (like the spine) and
  **re-measures** the torso frame before solving the legs / CCD / neck / tail (whether a
  chain moves the torso is auto-detected from the bone hierarchy). The back uses spine FK
  **or** trunk CCD, not both.
- **Leg toe hybrid**: the leg plants the paw (joints 9/13/19/23) with two-bone IK, then an
  optional toe bone is added **on top with FK** along the SMAL paw→toe segment (joints
  10/14/20/24). The toe is a leaf, so setting its world rotation does not disturb the
  planted paw — it keeps the contact while adding toe detail. Leave `RetargetLeg.toeBone`
  empty to skip.
- **Jaw / ear rotation copy**: SMAL joints 32 (jaw) / 33 (left ear) / 34 (right ear) have
  no length, so only their **local rotation** is copied onto the target bone
  (`RetargetCopyRotation`). It transfers **only the change from neutral (rest), not the raw
  rotation** — the reference rest pose is the value `SmalMotionPlayer` captured **before**
  posing frame 0 (`TryGetRestLocalRotation`) (falls back to frame 0 with a warning if the
  source has no player). The local delta vs SMAL neutral is taken as an axis-angle, the axis
  is remapped, then the angle is multiplied by the **`Weight`** and added to the target bind
  → axis re-placement + strength control. If the rigs' axes differ, `Map X/Y/Z` (enum
  `Axis`) maps SMAL x/y/z rotation onto the target's ± axes (mirror left/right with opposite
  signs). `Weight` = 1 is 1:1, `<1` suppresses, 0 freezes, `>1` exaggerates. The angle is
  normalized to the shortest arc (−180..180) before scaling, so the multiplier is exactly
  proportional to the real rotation. Being local, it is independent of the head pose. On
  load, `Weight<=0` is corrected to 1 (so a new element starting at 0 copies 1:1) — set 0 in
  Play to fully freeze.
- **Execution order**: the retarget drivers (`SmalRetargeter`/`SmalIkTargetDriver`) must
  read the source SMAL rig after it has been posed, so ordering is guaranteed with
  `LateUpdate` / `DefaultExecutionOrder`.
- **Asset dependencies**: SMAL clips need the imported rig named `SMAL_joint_NN`;
  `SmalIkTargetDriver` needs the target rig's `RigBuilder` (Animation Rigging) to be active.
