using System;
using System.Collections.Generic;
using UnityEngine;

namespace PetDemo
{
    /// <summary>
    /// Same retarget as <see cref="SmalRetargeter"/>, but driven straight from a clip
    /// JSON (the one <c>Python/export_smal_unity_clip.py</c> produces) instead of from a
    /// live <see cref="SmalMotionPlayer"/> posing a separate source rig. There is no SMAL
    /// FBX in the scene: the SMAL joint positions are reconstructed each frame by forward
    /// kinematics from the clip's rest skeleton (<c>restJoints</c> + <c>parents</c>) and
    /// its per-frame world-space rotation deltas, so the retarget needs no source
    /// skeleton at all.
    ///
    /// Why this works without the calibration that <see cref="SmalMotionPlayer"/> does:
    /// every goal in the retarget is built in the torso frame and normalized by the
    /// part's own length, so the SMAL body's absolute orientation and scale cancel out.
    /// The reconstructed joints can therefore stay in raw SMAL/armature space — no fit
    /// to an imported rig is required. An optional <see cref="smalSpaceRoot"/> lets you
    /// place/orient/scale that virtual SMAL space in the scene (needed only when
    /// <see cref="applyGlobalMotion"/> overlays the captured body translation/turn onto
    /// the target).
    ///
    /// Caveats vs. the rig path:
    ///   * Copy-rotation (jaw / ears) uses the joint's world delta-from-rest in SMAL
    ///     (armature) space, not the FBX bone's local frame, so the per-axis remap
    ///     (mapX/Y/Z) will generally need re-tuning from the rig-path values.
    ///   * <see cref="applyGlobalMotion"/> (target follows the captured body motion) is
    ///     meaningful only with <see cref="applySourceGlobalMotion"/> on and a sensible
    ///     <see cref="smalSpaceRoot"/> placement.
    ///
    /// The FK math and joint reconstruction are the only additions; the solve pipeline
    /// (torso frame, two-bone leg IK, CCD trunk, per-segment spine/neck/tail FK, toe FK,
    /// copy-rotation, foot lock, grounding) is identical to <see cref="SmalRetargeter"/>.
    /// Runs in LateUpdate: advances its own playback clock, reconstructs the SMAL pose,
    /// then retargets. Attach to the target root and assign the clip JSON.
    /// </summary>
    public class SmalRetargeterDirect : MonoBehaviour, IBodyFrameProvider, IGroundable
    {
        [Header("Source clip")]
        [Tooltip("JSON produced by Python/export_smal_unity_clip.py.")]
        [SerializeField] TextAsset clipJson;
        [Tooltip("SMAL rig in its REST (bind) pose, read ONCE at startup to recover the " +
                 "SMAL->Unity coordinate map (rotation + handedness + scale) that the raw " +
                 "clip does not carry. The rig is never driven — only its rest bone " +
                 "positions are fitted — so you can leave it inactive/hidden. WITHOUT it the " +
                 "joints stay in raw armature space and the legs flip (Unity uses the " +
                 "opposite handedness, which the torso frame cannot cancel). Strongly " +
                 "recommended.")]
        [SerializeField] Transform calibrationRig;
        [Tooltip("Optional transform whose placement/orientation/scale positions the " +
                 "reconstructed SMAL skeleton in the scene. Used only when no Calibration " +
                 "Rig is set (raw armature space). Ignored once a Calibration Rig is fitted.")]
        [SerializeField] Transform smalSpaceRoot;

        [Header("Playback")]
        [Tooltip("0 = use the clip's own fps.")]
        [SerializeField] float fpsOverride;
        [SerializeField] bool loop = true;
        [Tooltip("Bake the clip's captured global body transform (turn / translate / " +
                 "scale) into the reconstructed joint positions. Disable to keep the SMAL " +
                 "body planted and reconstruct only its articulation.")]
        [SerializeField] bool applySourceGlobalMotion = true;

        [Header("Bone name prefix (informational)")]
        [Tooltip("Only used to size joint arrays; the clip carries its own jointCount.")]
        [SerializeField] string boneNamePrefix = "SMAL_joint_";

        [Header("Rig")]
        [Tooltip("Root of the target rig to drive. Defaults to this transform.")]
        [SerializeField] Transform targetRoot;

        [Header("Torso frame bones (Target)")]
        [SerializeField] Transform leftShoulderBone;
        [SerializeField] Transform rightShoulderBone;
        [SerializeField] Transform leftHipBone;
        [SerializeField] Transform rightHipBone;

        [Header("Legs")]
        [Tooltip("IK: bend legs so the paws reach the mapped SMAL foot positions (feet planted). " +
                 "FK: transfer the SMAL leg-segment directions onto the bones (feet may float). " +
                 "The trunk always uses CCD.")]
        [SerializeField] RetargetMode legMode = RetargetMode.IK;
        [Tooltip("Max twist (roll about the bone's length axis) kept on the leg IK bones, in " +
                 "degrees. 0 = fully locked to the rest roll. Only affects IK mode.")]
        [SerializeField, Range(0f, 90f)] float maxLegRollDegrees;
        [Tooltip("Ease the last fraction of leg extension so the knee approaches full " +
                 "straight smoothly instead of snapping. 0 = off. IK / FK-plant only.")]
        [SerializeField, Range(0f, 0.2f)] float ikSoftZone = 0.03f;
        [SerializeField] RetargetLeg[] legs = SmalRetargetMap.DefaultLegs();

        [Header("Trunk / head (CCD, optional)")]
        [Tooltip("CCD chains for a single end goal (e.g. head aim). Use EITHER a CCD trunk " +
                 "OR the spine FK chain below for the back, not both on the same bones.")]
        [SerializeField] RetargetChain[] chains = SmalRetargetMap.DefaultChains();
        [SerializeField] int ikIterations = 10;

        [Header("Spine / neck / tail (per-segment FK)")]
        [Tooltip("Each chain's bones follow the SMAL segment directions through the body " +
                 "frame. Rotation only, so the chain keeps its own length (any length " +
                 "difference is absorbed) and never twists — the whole curve is reproduced. " +
                 "Name a chain spine/neck/tail to auto-fill its SMAL joints. Leave empty to skip.")]
        [SerializeField] RetargetFkChain[] fkChains = SmalRetargetMap.DefaultFkChains();

        [Header("Copy rotation (jaw / ears)")]
        [Tooltip("Copy a SMAL joint's world delta-from-rest (jaw 32, ears 33/34) onto a " +
                 "target bone. Rotation only; use the per-axis map to reorient it. NOTE: the " +
                 "delta is in SMAL armature space here, so the remap generally differs from " +
                 "the rig-path values. Leave the target bone empty to skip.")]
        [SerializeField] RetargetCopyRotation[] copyRotations = SmalRetargetMap.DefaultCopyRotations();

        [Header("Global motion (target overlay)")]
        [Tooltip("Overlay the target body onto the SMAL body (follow its turn/translation). " +
                 "Needs Apply Source Global Motion on to have anything to follow. " +
                 "Leave off to keep the target planted and only play the articulation.")]
        [SerializeField] bool applyGlobalMotion;
        [Tooltip("Which world axes the body-overlay rotation may act on.")]
        [SerializeField] RootAxis globalRotationAxes = RootAxis.All;
        [Tooltip("Which world axes the body-overlay translation may act on.")]
        [SerializeField] RootAxis globalTranslationAxes = RootAxis.All;

        [Header("Smoothing (One-Euro on source joints)")]
        [Tooltip("Adaptively low-pass the reconstructed SMAL joint positions before " +
                 "retargeting: strong smoothing when slow, no lag when fast.")]
        [SerializeField] bool smoothSource = true;
        [Tooltip("Base cutoff (Hz). Lower = smoother but laggier at rest.")]
        [SerializeField] float sourceMinCutoff = 1.5f;
        [Tooltip("Speed coefficient. Higher = follows fast motion with less lag.")]
        [SerializeField] float sourceBeta = 0.02f;

        [Header("Foot contact lock (anti-skate)")]
        [Tooltip("Freeze a paw in world space while it is planted so it stops sliding. " +
                 "In FK mode this adds a light IK correction only during stance.")]
        [SerializeField] bool lockPlantedFeet = true;
        [Tooltip("Enter stance when the source foot's speed drops below this (m/s).")]
        [SerializeField] float footLockSpeed = 0.03f;
        [Tooltip("Leave stance when the source foot's speed rises above this (m/s).")]
        [SerializeField] float footUnlockSpeed = 0.1f;
        [Tooltip("Ease time for a plant / release, seconds.")]
        [SerializeField] float footLockAttack = 0.08f;

        [Header("Debug")]
        [SerializeField] bool logCalibration = true;
        [SerializeField] bool drawGizmos = true;
        [SerializeField] float gizmoRadius = 0.02f;

        const float Eps = 1e-8f;

        // ---- Clip / source reconstruction ----------------------------------------

        [Serializable]
        class Clip
        {
            public string name;
            public int fps;
            public int frameCount;
            public int jointCount;
            public string boneNamePrefix;
            public int[] parents;
            public float[] restJoints; // jointCount * 3
            public float[] deltas;     // frameCount * jointCount * 4  (x,y,z,w)
            public float[] rootRot;    // frameCount * 4               (x,y,z,w)
            public float[] rootPos;    // frameCount * 3
            public float[] rootScale;  // frameCount
        }

        Clip clip;
        Vector3[] restJoint;           // [joint] rest position (armature space)
        int[] jointParent;             // [joint] parent index (-1 = root)
        Quaternion[] armDelta;         // [joint] current-frame world delta-from-rest (arm space)
        Vector3[] smalWorldPos;        // [joint] current-frame reconstructed position (scene/world)
        float clipTime;                // playback clock

        // Recovered SMAL(arm) -> calibrationRig-local map (A = scale*R linear, b = offset),
        // fitted once from the rig's rest bone positions. Composed with the rig's own
        // world transform each frame to place the reconstructed joints in the scene.
        Matrix4x4 armToRoot = Matrix4x4.identity;
        bool calibrated;

        // ---- Target rig (same as SmalRetargeter) ---------------------------------

        Transform[] legUpper, legLower, legTip;
        float[] legLenTarget;
        Quaternion[] fkCalUpper, fkCalLower;
        Quaternion[] bindLocalUpper, bindLocalLower;
        Vector3[] rollAxisUpper, rollAxisLower;
        Transform[] legToe;
        Vector3[] toeRefDir;
        Quaternion[] toeBindRel;
        Transform[][] chainBones;
        Transform[] chainEffectors;
        float[] chainLenTarget;
        Transform tLsh, tRsh, tLhip, tRhip;
        Transform[][] fkBones;
        int[][] fkSegIndex;
        Vector3[][] fkRefDirT;
        Quaternion[][] fkBindRel;
        bool[] fkPreBody;
        Transform[] copyBone;
        Quaternion[] copyTargetBind;
        Vector3[] copyRemapX, copyRemapY, copyRemapZ;

        Transform[] allBones;
        Quaternion[] bindLocalRot;
        Vector3[] bindLocalPos;
        Vector3 rootBindLocalPos;
        Quaternion rootBindLocalRot;

        Vector3[] gizmoTargets;
        Vector3[] gizmoPoles;
        Vector3[] legBendAxis;

        OneEuroFilterVector3[] smalFilters;
        Vector3[] smalPosFrame;
        bool[] smalPosValid;
        FootPlanter[] footPlanters;
        Vector3[] prevLegNormalSmal;
        Vector3[] lastPole;
        Quaternion lastInvSRot = Quaternion.identity;

        void Awake()
        {
            if (targetRoot == null)
                targetRoot = transform;
            if (clipJson == null)
                throw new MissingReferenceException("SmalRetargeterDirect: no clip JSON assigned");

            clip = JsonUtility.FromJson<Clip>(clipJson.text);
            if (clip == null || clip.frameCount <= 0 || clip.jointCount <= 0)
                throw new InvalidOperationException("SmalRetargeterDirect: clip JSON failed to parse");
            if (clip.parents == null || clip.parents.Length < clip.jointCount
                || clip.restJoints == null || clip.restJoints.Length < clip.jointCount * 3)
                throw new InvalidOperationException(
                    "SmalRetargeterDirect: clip is missing parents / restJoints (re-export the clip)");

            int n = clip.jointCount;
            restJoint = new Vector3[n];
            jointParent = new int[n];
            for (int j = 0; j < n; j++)
            {
                restJoint[j] = new Vector3(
                    clip.restJoints[j * 3 + 0], clip.restJoints[j * 3 + 1], clip.restJoints[j * 3 + 2]);
                jointParent[j] = clip.parents[j];
            }
            armDelta = new Quaternion[n];
            smalWorldPos = new Vector3[n];

            // Same Inspector-defaulting as SmalRetargeter.
            if (fkChains == null)
                fkChains = Array.Empty<RetargetFkChain>();
            for (int i = 0; i < fkChains.Length; i++)
                if ((fkChains[i].joints == null || fkChains[i].joints.Length < 2)
                    && fkChains[i].bones != null && fkChains[i].bones.Length > 0)
                    fkChains[i].joints = SmalRetargetMap.DefaultJoints(fkChains[i].name);

            for (int i = 0; i < legs.Length; i++)
                if (legs[i].toeBone != null && legs[i].toeJoint <= 0)
                    legs[i].toeJoint = legs[i].targetJoint + 1;

            if (copyRotations == null)
                copyRotations = Array.Empty<RetargetCopyRotation>();
            for (int i = 0; i < copyRotations.Length; i++)
                if (copyRotations[i].weight <= 0f)
                    copyRotations[i].weight = 1f;

            ResolveTargetRig();
            CacheBind();

            smalFilters = new OneEuroFilterVector3[n];
            smalPosFrame = new Vector3[n];
            smalPosValid = new bool[n];
            for (int j = 0; j < n; j++)
                smalFilters[j] = new OneEuroFilterVector3(sourceMinCutoff, sourceBeta);

            foreach (var smr in targetRoot.GetComponentsInChildren<SkinnedMeshRenderer>())
                smr.updateWhenOffscreen = true;
            foreach (var animator in targetRoot.GetComponentsInChildren<Animator>())
                animator.enabled = false;

            if (calibrationRig != null)
                CalibrateFromRig();

            // Reconstruct frame 0 so Start's bind calibration reads valid joint positions.
            ComputePose(0, 0, 0f);

            if (logCalibration)
                Debug.Log($"SmalRetargeterDirect: loaded clip '{clip.name}' " +
                          $"({clip.frameCount} frames, {n} joints, fps={clip.fps})");
        }

        void Start()
        {
            // Rest lengths (constant under a rotation-only solve). Goals are normalized to
            // these so the target keeps its size.
            legLenTarget = new float[legs.Length];
            for (int i = 0; i < legs.Length; i++)
                legLenTarget[i] = (legLower[i].position - legUpper[i].position).magnitude
                                + (legTip[i].position - legLower[i].position).magnitude;

            chainLenTarget = new float[chains.Length];
            for (int i = 0; i < chains.Length; i++)
                chainLenTarget[i] = (chainEffectors[i].position - chainBones[i][0].position).magnitude;

            fkCalUpper = new Quaternion[legs.Length];
            fkCalLower = new Quaternion[legs.Length];
            footPlanters = new FootPlanter[legs.Length];
            prevLegNormalSmal = new Vector3[legs.Length];
            lastPole = new Vector3[legs.Length];
            for (int i = 0; i < legs.Length; i++)
            {
                footPlanters[i] = new FootPlanter();
                lastPole[i] = legLower[i].position;
            }
            TorsoFrame(SmalPos(SmalRetargetMap.LeftShoulder), SmalPos(SmalRetargetMap.RightShoulder),
                       SmalPos(SmalRetargetMap.LeftHip), SmalPos(SmalRetargetMap.RightHip),
                       out Quaternion sRot0, out _, out _);
            TorsoFrame(tLsh.position, tRsh.position, tLhip.position, tRhip.position,
                       out Quaternion tRot0, out _, out _);
            for (int i = 0; i < legs.Length; i++)
            {
                LegDirs(legs[i], sRot0, tRot0, out Vector3 dU, out Vector3 dL, out Vector3 nrm);
                fkCalUpper[i] = Quaternion.Inverse(Quaternion.LookRotation(dU, nrm)) * legUpper[i].rotation;
                fkCalLower[i] = Quaternion.Inverse(Quaternion.LookRotation(dL, nrm)) * legLower[i].rotation;
                prevLegNormalSmal[i] = nrm.normalized;
            }

            bindLocalUpper = new Quaternion[legs.Length];
            bindLocalLower = new Quaternion[legs.Length];
            rollAxisUpper = new Vector3[legs.Length];
            rollAxisLower = new Vector3[legs.Length];
            for (int i = 0; i < legs.Length; i++)
            {
                bindLocalUpper[i] = legUpper[i].localRotation;
                bindLocalLower[i] = legLower[i].localRotation;
                rollAxisUpper[i] = (Quaternion.Inverse(legUpper[i].rotation)
                                    * (legLower[i].position - legUpper[i].position)).normalized;
                rollAxisLower[i] = (Quaternion.Inverse(legLower[i].rotation)
                                    * (legTip[i].position - legLower[i].position)).normalized;
            }

            toeRefDir = new Vector3[legs.Length];
            toeBindRel = new Quaternion[legs.Length];
            {
                Quaternion invSRot0 = Quaternion.Inverse(sRot0);
                Quaternion invTRot0 = Quaternion.Inverse(tRot0);
                for (int i = 0; i < legs.Length; i++)
                {
                    if (legToe[i] == null)
                        continue;
                    Vector3 segDir = SmalPos(legs[i].toeJoint) - SmalPos(legs[i].targetJoint);
                    if (segDir.sqrMagnitude < Eps)
                        throw new InvalidOperationException(
                            $"SmalRetargeterDirect: leg '{legs[i].name}' toe segment is degenerate at bind");
                    toeRefDir[i] = (invSRot0 * segDir).normalized;
                    toeBindRel[i] = invTRot0 * legToe[i].rotation;
                }
            }

            fkSegIndex = new int[fkChains.Length][];
            fkRefDirT = new Vector3[fkChains.Length][];
            fkBindRel = new Quaternion[fkChains.Length][];
            for (int i = 0; i < fkChains.Length; i++)
            {
                var bones = fkBones[i];
                fkSegIndex[i] = new int[bones.Length];
                fkRefDirT[i] = new Vector3[bones.Length];
                fkBindRel[i] = new Quaternion[bones.Length];
                if (bones.Length == 0)
                    continue;

                int[] joints = fkChains[i].joints;
                int segs = joints.Length - 1;
                Quaternion invSRot0 = Quaternion.Inverse(sRot0);
                Quaternion invTRot0 = Quaternion.Inverse(tRot0);
                for (int k = 0; k < bones.Length; k++)
                {
                    int seg = Mathf.Clamp(
                        Mathf.FloorToInt((k + 0.5f) * segs / bones.Length), 0, segs - 1);
                    Vector3 segDir = SmalPos(joints[seg + 1]) - SmalPos(joints[seg]);
                    if (segDir.sqrMagnitude < Eps)
                        throw new InvalidOperationException(
                            $"SmalRetargeterDirect: FK chain '{fkChains[i].name}' segment {seg} is degenerate at bind");
                    fkSegIndex[i][k] = seg;
                    fkRefDirT[i][k] = (invSRot0 * segDir).normalized;
                    fkBindRel[i][k] = invTRot0 * bones[k].rotation;
                }
            }

            // Copy-rotation: the clip's per-joint delta is already relative to the rest
            // pose, so the neutral reference is identity — only the target bind and the
            // axis remap are recorded here.
            copyTargetBind = new Quaternion[copyRotations.Length];
            copyRemapX = new Vector3[copyRotations.Length];
            copyRemapY = new Vector3[copyRotations.Length];
            copyRemapZ = new Vector3[copyRotations.Length];
            for (int i = 0; i < copyRotations.Length; i++)
            {
                if (copyBone[i] == null)
                    continue;
                var cr = copyRotations[i];
                copyTargetBind[i] = copyBone[i].localRotation;
                copyRemapX[i] = RetargetCopyRotation.AxisVec(cr.mapX);
                copyRemapY[i] = RetargetCopyRotation.AxisVec(cr.mapY);
                copyRemapZ[i] = RetargetCopyRotation.AxisVec(cr.mapZ);
            }

            if (logCalibration)
                Debug.Log($"SmalRetargeterDirect: rest leg lengths=[{string.Join(", ", legLenTarget)}]");
        }

        // ---- Source reconstruction (clip -> joint positions) ---------------------

        /// <summary>Advance the playback clock and reconstruct this frame's SMAL pose.</summary>
        void Tick()
        {
            float fps = fpsOverride > 0f ? fpsOverride : clip.fps;
            clipTime += Time.deltaTime;
            float framePos = clipTime * fps;
            int f = Mathf.FloorToInt(framePos);
            float frac = framePos - f;
            int next;
            if (loop)
            {
                f = ((f % clip.frameCount) + clip.frameCount) % clip.frameCount;
                next = (f + 1) % clip.frameCount;
            }
            else
            {
                f = Mathf.Min(f, clip.frameCount - 1);
                next = Mathf.Min(f + 1, clip.frameCount - 1);
                if (f == next)
                    frac = 0f;
            }
            ComputePose(f, next, frac);
        }

        /// <summary>
        /// Reconstruct every joint's world position from the clip. The clip stores a
        /// world-space rotation delta-from-rest per joint (in armature space), so a joint
        /// does not accumulate its ancestors' rotations: the segment parent->child is
        /// rotated by the PARENT's delta alone. Positions therefore come from
        ///   pos[child] = pos[parent] + delta[parent] * (rest[child] - rest[parent])
        /// walked in joint order (parents precede children in SMAL order). The captured
        /// global body transform is optionally composed on top, then the whole skeleton
        /// is placed into the scene through <see cref="smalSpaceRoot"/> (identity if none).
        /// </summary>
        void ComputePose(int f, int next, float frac)
        {
            int n = clip.jointCount;
            int baseF = f * n * 4;
            int baseN = next * n * 4;
            for (int j = 0; j < n; j++)
            {
                int i = baseF + j * 4;
                var d = new Quaternion(clip.deltas[i + 0], clip.deltas[i + 1],
                                       clip.deltas[i + 2], clip.deltas[i + 3]);
                if (frac > 0f)
                {
                    int m = baseN + j * 4;
                    var dNext = new Quaternion(clip.deltas[m + 0], clip.deltas[m + 1],
                                               clip.deltas[m + 2], clip.deltas[m + 3]);
                    d = Quaternion.Slerp(d, dNext, frac);
                }
                armDelta[j] = d;
            }

            // FK in armature space.
            for (int j = 0; j < n; j++)
            {
                int p = jointParent[j];
                if (p < 0)
                    smalWorldPos[j] = restJoint[j];
                else
                    smalWorldPos[j] = smalWorldPos[p] + armDelta[p] * (restJoint[j] - restJoint[p]);
            }

            // Captured global body transform (armature space): rotate/scale/translate the
            // whole skeleton. Copy-rotation stays on the pre-global per-joint delta, so it
            // is unaffected — matching the rig path where the joint's LOCAL rotation is
            // independent of the body's global motion.
            Matrix4x4 objArm = Matrix4x4.identity;
            if (applySourceGlobalMotion)
            {
                var gRot = new Quaternion(clip.rootRot[f * 4 + 0], clip.rootRot[f * 4 + 1],
                                          clip.rootRot[f * 4 + 2], clip.rootRot[f * 4 + 3]);
                var gPos = new Vector3(clip.rootPos[f * 3 + 0], clip.rootPos[f * 3 + 1],
                                       clip.rootPos[f * 3 + 2]);
                float gScale = clip.rootScale[f];
                if (frac > 0f)
                {
                    var gRotN = new Quaternion(clip.rootRot[next * 4 + 0], clip.rootRot[next * 4 + 1],
                                               clip.rootRot[next * 4 + 2], clip.rootRot[next * 4 + 3]);
                    var gPosN = new Vector3(clip.rootPos[next * 3 + 0], clip.rootPos[next * 3 + 1],
                                            clip.rootPos[next * 3 + 2]);
                    gRot = Quaternion.Slerp(gRot, gRotN, frac);
                    gPos = Vector3.Lerp(gPos, gPosN, frac);
                    gScale = Mathf.Lerp(gScale, clip.rootScale[next], frac);
                }
                objArm = Matrix4x4.TRS(gPos, gRot, new Vector3(gScale, gScale, gScale));
            }

            if (calibrated)
            {
                // Recovered SMAL->Unity map: arm -> calibrationRig-local (armToRoot) ->
                // world (the rig's own transform). This reproduces exactly the space that
                // SmalMotionPlayer poses its bones in — rotation, handedness and scale — so
                // the legs no longer flip. The rig is read only for its placement; its bones
                // are never driven. Global body motion (objArm) is applied in arm space
                // before the map, matching SmalMotionPlayer's conjugation.
                Matrix4x4 xform = calibrationRig.localToWorldMatrix * armToRoot * objArm;
                for (int j = 0; j < n; j++)
                    smalWorldPos[j] = xform.MultiplyPoint3x4(smalWorldPos[j]);
            }
            else
            {
                // No calibration rig: raw armature space (the legs flip when the clip's
                // handedness differs from Unity's), optionally placed via smalSpaceRoot.
                if (applySourceGlobalMotion)
                    for (int j = 0; j < n; j++)
                        smalWorldPos[j] = objArm.MultiplyPoint3x4(smalWorldPos[j]);
                if (smalSpaceRoot != null)
                    for (int j = 0; j < n; j++)
                        smalWorldPos[j] = smalSpaceRoot.TransformPoint(smalWorldPos[j]);
            }
        }

        /// <summary>
        /// Fit the affine map (A, b) with bonePos_in_rigLocal ≈ A * restJoint + b from the
        /// clip's exported rest skeleton to the calibration rig's actual rest bone
        /// positions (least squares). This recovers the SMAL->Unity convention — rotation,
        /// handedness (A may be a reflection) and scale — that the raw clip omits, the same
        /// fit <see cref="SmalMotionPlayer"/> does. Read once at startup; the rig is never
        /// posed.
        /// </summary>
        void CalibrateFromRig()
        {
            int n = clip.jointCount;
            var bones = new Transform[n];
            var all = calibrationRig.GetComponentsInChildren<Transform>(true);
            for (int j = 0; j < n; j++)
            {
                string wanted = $"{boneNamePrefix}{j:00}";
                foreach (var t in all)
                    if (t.name == wanted) { bones[j] = t; break; }
                if (bones[j] == null)
                    throw new MissingReferenceException(
                        $"SmalRetargeterDirect: calibration rig bone '{wanted}' not found under " +
                        $"{calibrationRig.name}");
            }

            var arm = new Vector3[n];
            var rootLocal = new Vector3[n];
            for (int j = 0; j < n; j++)
            {
                arm[j] = restJoint[j];
                rootLocal[j] = calibrationRig.InverseTransformPoint(bones[j].position);
            }

            // Normal equations per output axis: minimise ||rootLocal_k - (A_k·arm + b_k)||.
            double[,] N = new double[4, 4];
            double[,] rhs = new double[4, 3];
            for (int j = 0; j < n; j++)
            {
                double[] p = { arm[j].x, arm[j].y, arm[j].z, 1.0 };
                for (int r = 0; r < 4; r++)
                {
                    for (int c = 0; c < 4; c++) N[r, c] += p[r] * p[c];
                    rhs[r, 0] += p[r] * rootLocal[j].x;
                    rhs[r, 1] += p[r] * rootLocal[j].y;
                    rhs[r, 2] += p[r] * rootLocal[j].z;
                }
            }
            double[,] sol = SolveLinear(N, rhs);

            armToRoot = Matrix4x4.identity;
            for (int axis = 0; axis < 3; axis++)
            {
                armToRoot[axis, 0] = (float)sol[0, axis];
                armToRoot[axis, 1] = (float)sol[1, axis];
                armToRoot[axis, 2] = (float)sol[2, axis];
                armToRoot[axis, 3] = (float)sol[3, axis];
            }
            calibrated = true;

            if (logCalibration)
            {
                float maxErr = 0f;
                for (int j = 0; j < n; j++)
                    maxErr = Mathf.Max(maxErr, (armToRoot.MultiplyPoint3x4(arm[j]) - rootLocal[j]).magnitude);
                Debug.Log($"SmalRetargeterDirect: calibrated from rig '{calibrationRig.name}' " +
                          $"(max rest fit error={maxErr:F5} m)");
            }
        }

        /// <summary>Gaussian elimination with partial pivoting: solves A (4x4) X = B (4xm).</summary>
        static double[,] SolveLinear(double[,] a, double[,] b)
        {
            int n = 4, m = b.GetLength(1);
            var M = new double[n, n + m];
            for (int r = 0; r < n; r++)
            {
                for (int c = 0; c < n; c++) M[r, c] = a[r, c];
                for (int c = 0; c < m; c++) M[r, n + c] = b[r, c];
            }
            for (int col = 0; col < n; col++)
            {
                int pivot = col;
                for (int r = col + 1; r < n; r++)
                    if (Math.Abs(M[r, col]) > Math.Abs(M[pivot, col])) pivot = r;
                if (Math.Abs(M[pivot, col]) < 1e-12)
                    throw new InvalidOperationException("SmalRetargeterDirect: calibration is singular");
                if (pivot != col)
                    for (int c = 0; c < n + m; c++)
                        (M[col, c], M[pivot, c]) = (M[pivot, c], M[col, c]);
                double d = M[col, col];
                for (int c = 0; c < n + m; c++) M[col, c] /= d;
                for (int r = 0; r < n; r++)
                {
                    if (r == col) continue;
                    double factor = M[r, col];
                    for (int c = 0; c < n + m; c++) M[r, c] -= factor * M[col, c];
                }
            }
            var x = new double[n, m];
            for (int r = 0; r < n; r++)
                for (int c = 0; c < m; c++) x[r, c] = M[r, n + c];
            return x;
        }

        void ResolveTargetRig()
        {
            Transform Require(Transform bone, string mapping)
            {
                if (bone == null)
                    throw new MissingReferenceException(
                        $"SmalRetargeterDirect: target mapping '{mapping}' has no bone assigned");
                if (bone != targetRoot && !bone.IsChildOf(targetRoot))
                    throw new InvalidOperationException(
                        $"SmalRetargeterDirect: mapped bone '{bone.name}' is not under " +
                        $"target root '{targetRoot.name}'");
                return bone;
            }

            tLsh = Require(leftShoulderBone, "Left Shoulder");
            tRsh = Require(rightShoulderBone, "Right Shoulder");
            tLhip = Require(leftHipBone, "Left Hip");
            tRhip = Require(rightHipBone, "Right Hip");

            legUpper = new Transform[legs.Length];
            legLower = new Transform[legs.Length];
            legTip = new Transform[legs.Length];
            legToe = new Transform[legs.Length];
            for (int i = 0; i < legs.Length; i++)
            {
                var leg = legs[i];
                legUpper[i] = Require(leg.upperBone, $"{leg.name} Upper");
                legLower[i] = Require(leg.lowerBone, $"{leg.name} Lower");
                legTip[i] = Require(leg.tipBone, $"{leg.name} Tip");
                RequireDescendant(legUpper[i], legLower[i], leg.name);
                RequireDescendant(legLower[i], legTip[i], leg.name);
                if (leg.toeBone != null)
                {
                    legToe[i] = Require(leg.toeBone, $"{leg.name} Toe");
                    RequireDescendant(legTip[i], legToe[i], leg.name);
                }
            }

            chainBones = new Transform[chains.Length][];
            chainEffectors = new Transform[chains.Length];
            for (int i = 0; i < chains.Length; i++)
            {
                Transform baseBone = Require(chains[i].baseBone, $"{chains[i].name} Base");
                chainEffectors[i] = Require(chains[i].effectorBone, $"{chains[i].name} Effector");
                chainBones[i] = BuildChain(baseBone, chainEffectors[i], chains[i].name);
            }

            fkBones = new Transform[fkChains.Length][];
            fkPreBody = new bool[fkChains.Length];
            for (int i = 0; i < fkChains.Length; i++)
            {
                var fk = fkChains[i];
                if (fk.bones == null || fk.bones.Length == 0)
                {
                    fkBones[i] = Array.Empty<Transform>();
                    continue;
                }
                if (fk.joints == null || fk.joints.Length < 2)
                    throw new MissingReferenceException(
                        $"SmalRetargeterDirect: FK chain '{fk.name}' needs at least two SMAL joints " +
                        "(name it spine/neck/tail to auto-fill, or set them explicitly)");

                var bones = new Transform[fk.bones.Length];
                for (int k = 0; k < bones.Length; k++)
                {
                    bones[k] = Require(fk.bones[k], $"{fk.name} bone {k}");
                    if (k > 0)
                        RequireDescendant(bones[k - 1], bones[k], fk.name);
                }
                fkBones[i] = bones;
                fkPreBody[i] = MovesTorsoFrame(bones);
            }

            copyBone = new Transform[copyRotations.Length];
            for (int i = 0; i < copyRotations.Length; i++)
                if (copyRotations[i].targetBone != null)
                    copyBone[i] = Require(copyRotations[i].targetBone, $"{copyRotations[i].name} target");
        }

        bool MovesTorsoFrame(Transform[] bones)
        {
            foreach (Transform bone in bones)
                if (tLsh.IsChildOf(bone) || tRsh.IsChildOf(bone)
                    || tLhip.IsChildOf(bone) || tRhip.IsChildOf(bone))
                    return true;
            return false;
        }

        static void RequireDescendant(Transform ancestor, Transform descendant, string name)
        {
            for (Transform t = descendant.parent; t != null; t = t.parent)
                if (t == ancestor) return;
            throw new MissingReferenceException(
                $"SmalRetargeterDirect: leg '{name}' bones are not a parent->child chain " +
                $"({ancestor.name} -> {descendant.name})");
        }

        static Transform[] BuildChain(Transform baseBone, Transform effector, string name)
        {
            var chain = new List<Transform>();
            for (Transform t = effector.parent; t != null; t = t.parent)
            {
                chain.Add(t);
                if (t == baseBone) break;
            }
            if (chain.Count == 0 || chain[chain.Count - 1] != baseBone)
                throw new MissingReferenceException(
                    $"SmalRetargeterDirect: chain '{name}' is not a contiguous chain from " +
                    $"{baseBone.name} to {effector.name}");
            chain.Reverse();
            return chain.ToArray();
        }

        void CacheBind()
        {
            allBones = targetRoot.GetComponentsInChildren<Transform>();
            bindLocalRot = new Quaternion[allBones.Length];
            bindLocalPos = new Vector3[allBones.Length];
            for (int i = 0; i < allBones.Length; i++)
            {
                bindLocalRot[i] = allBones[i].localRotation;
                bindLocalPos[i] = allBones[i].localPosition;
            }
            rootBindLocalPos = targetRoot.localPosition;
            rootBindLocalRot = targetRoot.localRotation;
        }

        void RestoreBind()
        {
            for (int i = 0; i < allBones.Length; i++)
            {
                allBones[i].localRotation = bindLocalRot[i];
                allBones[i].localPosition = bindLocalPos[i];
            }
            targetRoot.localPosition = rootBindLocalPos;
            targetRoot.localRotation = rootBindLocalRot;
        }

        void LateUpdate()
        {
            RestoreBind();

            // Advance the clock and rebuild the SMAL pose for this frame BEFORE reading it.
            Tick();

            if (smalPosValid != null)
                System.Array.Clear(smalPosValid, 0, smalPosValid.Length);

            TorsoFrame(SmalPos(SmalRetargetMap.LeftShoulder), SmalPos(SmalRetargetMap.RightShoulder),
                       SmalPos(SmalRetargetMap.LeftHip), SmalPos(SmalRetargetMap.RightHip),
                       out Quaternion sRot, out Vector3 sOrigin, out _);

            if (applyGlobalMotion)
                PlaceRoot(sRot, sOrigin);

            int gizmoCount = legs.Length + chains.Length + fkChains.Length;
            if (gizmoTargets == null || gizmoTargets.Length != gizmoCount)
                gizmoTargets = new Vector3[gizmoCount];
            if (gizmoPoles == null || gizmoPoles.Length != legs.Length)
                gizmoPoles = new Vector3[legs.Length];
            if (legBendAxis == null || legBendAxis.Length != legs.Length)
                legBendAxis = new Vector3[legs.Length];

            Quaternion invSRot = Quaternion.Inverse(sRot);
            lastInvSRot = invSRot;

            TorsoFrame(tLsh.position, tRsh.position, tLhip.position, tRhip.position,
                       out Quaternion tRotPre, out _, out _);
            for (int i = 0; i < fkChains.Length; i++)
                if (fkPreBody[i])
                    SolveFkChain(i, invSRot, tRotPre);

            TorsoFrame(tLsh.position, tRsh.position, tLhip.position, tRhip.position,
                       out Quaternion tRot, out _, out _);

            for (int i = 0; i < legs.Length; i++)
            {
                var leg = legs[i];
                if (legMode == RetargetMode.FK)
                {
                    LegDirs(leg, sRot, tRot, out Vector3 dU, out Vector3 dL, out Vector3 nrm);
                    nrm = StableNormal(ref prevLegNormalSmal[i], nrm);
                    legUpper[i].rotation = Quaternion.LookRotation(dU, nrm) * fkCalUpper[i];
                    legLower[i].rotation = Quaternion.LookRotation(dL, nrm) * fkCalLower[i];

                    if (lockPlantedFeet && footPlanters != null)
                    {
                        Vector3 detectFk = SmalPos(leg.targetJoint);
                        Vector3 plantFk = footPlanters[i].Filter(
                            legTip[i].position, detectFk, Time.deltaTime,
                            footLockSpeed, footUnlockSpeed, footLockAttack);
                        if ((plantFk - legTip[i].position).sqrMagnitude > 1e-10f)
                            TwoBoneIK.Solve(legUpper[i], legLower[i], legTip[i],
                                            plantFk, legLower[i].position, ikSoftZone);
                    }
                    lastPole[i] = legLower[i].position;
                    gizmoTargets[i] = legTip[i].position;
                    gizmoPoles[i] = legLower[i].position;
                    continue;
                }

                Vector3 sRoot = SmalPos(leg.rootJoint);
                Vector3 sPole = SmalPos(leg.poleJoint);
                Vector3 sFoot = SmalPos(leg.targetJoint);
                float srcLen = (sPole - sRoot).magnitude + (sFoot - sPole).magnitude;
                if (srcLen < Eps)
                    throw new InvalidOperationException($"SmalRetargeterDirect: leg '{leg.name}' collapsed");

                float legScale = legLenTarget[i] / srcLen;
                Vector3 anchor = legUpper[i].position;
                Vector3 target = MapOffset(sFoot, sRoot, sRot, tRot, anchor, legScale);
                Vector3 pole = MapOffset(sPole, sRoot, sRot, tRot, anchor, legScale);
                if (lockPlantedFeet && footPlanters != null)
                    target = footPlanters[i].Filter(
                        target, sFoot, Time.deltaTime,
                        footLockSpeed, footUnlockSpeed, footLockAttack);
                TwoBoneIK.Solve(legUpper[i], legLower[i], legTip[i], target, pole, ikSoftZone,
                                ref legBendAxis[i]);
                LockRoll(legUpper[i], bindLocalUpper[i], rollAxisUpper[i], maxLegRollDegrees);
                LockRoll(legLower[i], bindLocalLower[i], rollAxisLower[i], maxLegRollDegrees);
                lastPole[i] = pole;
                gizmoTargets[i] = target;
                gizmoPoles[i] = pole;
            }

            SolveToeFk(invSRot, tRot);

            for (int i = 0; i < chains.Length; i++)
            {
                var chain = chains[i];
                Vector3 sBase = SmalPos(chain.rootJoint);
                Vector3 sEnd = SmalPos(chain.endJoint);
                float srcLen = (sEnd - sBase).magnitude;
                if (srcLen < Eps)
                    throw new InvalidOperationException($"SmalRetargeterDirect: chain '{chain.name}' collapsed");

                float chainScale = chainLenTarget[i] / srcLen;
                Vector3 anchor = chainBones[i][0].position;
                Vector3 target = MapOffset(sEnd, sBase, sRot, tRot, anchor, chainScale);
                ChainIK.Solve(chainBones[i], chainEffectors[i], target, ikIterations);
                gizmoTargets[legs.Length + i] = target;
            }

            for (int i = 0; i < fkChains.Length; i++)
                if (!fkPreBody[i])
                    SolveFkChain(i, invSRot, tRot);

            for (int i = 0; i < copyRotations.Length; i++)
                SolveCopyRotation(i);
        }

        void SolveToeFk(Quaternion invSRot, Quaternion tRot)
        {
            for (int i = 0; i < legs.Length; i++)
            {
                if (legToe[i] == null)
                    continue;
                Vector3 segDir = SmalPos(legs[i].toeJoint) - SmalPos(legs[i].targetJoint);
                if (segDir.sqrMagnitude < Eps)
                    continue;
                Vector3 curDir = (invSRot * segDir).normalized;
                Quaternion swing = Quaternion.FromToRotation(toeRefDir[i], curDir);
                legToe[i].rotation = tRot * swing * toeBindRel[i];
            }
        }

        bool IsFrontLeg(int i)
        {
            string nm = legs[i].name != null ? legs[i].name.ToLowerInvariant() : string.Empty;
            if (nm.Contains("front") || nm.Contains("fore"))
                return true;
            if (nm.Contains("back") || nm.Contains("hind") || nm.Contains("rear"))
                return false;
            return i < legs.Length / 2;
        }

        public void ApplyFootGrounding(FootGroundingSettings settings, FootGroundingState state)
        {
            if (legTip == null || legTip.Length == 0)
                return;
            if (!TryGetBodyFrame(out Vector3 origin, out Vector3 forward))
                return;

            var glegs = new GroundLeg[legs.Length];
            for (int i = 0; i < legs.Length; i++)
            {
                int li = i;
                glegs[i] = new GroundLeg
                {
                    tip = legTip[li],
                    isFront = IsFrontLeg(li),
                    solveToTip = pos =>
                    {
                        TwoBoneIK.Solve(legUpper[li], legLower[li], legTip[li],
                                        pos, lastPole[li], ikSoftZone, ref legBendAxis[li]);
                        LockRoll(legUpper[li], bindLocalUpper[li], rollAxisUpper[li], maxLegRollDegrees);
                        LockRoll(legLower[li], bindLocalLower[li], rollAxisLower[li], maxLegRollDegrees);
                    },
                };
            }

            FootGrounding.Solve(targetRoot, glegs, Vector3.up, origin, forward,
                                settings, state, Time.deltaTime);

            TorsoFrame(tLsh.position, tRsh.position, tLhip.position, tRhip.position,
                       out Quaternion tRot, out _, out _);
            SolveToeFk(lastInvSRot, tRot);
        }

        /// <summary>Copy one SMAL joint's world delta-from-rest onto its target bone. The
        /// clip's per-joint delta is already relative to the rest pose (neutral =
        /// identity), so it is used directly as the axis-angle articulation: remap the
        /// axis into the target bone's frame, scale by the weight, add to the target bind.
        /// NOTE: this delta is in SMAL armature space, so the remap differs from the FBX
        /// rig-path values.</summary>
        void SolveCopyRotation(int i)
        {
            if (copyBone[i] == null)
                return;

            Quaternion deltaS = armDelta[copyRotations[i].joint];
            deltaS.ToAngleAxis(out float angle, out Vector3 axisS);
            if (angle > 180f)
                angle -= 360f;

            Vector3 axisT = axisS.x * copyRemapX[i] + axisS.y * copyRemapY[i] + axisS.z * copyRemapZ[i];
            Quaternion deltaT = Quaternion.AngleAxis(angle * copyRotations[i].weight, axisT);
            copyBone[i].localRotation = copyTargetBind[i] * deltaT;
        }

        void SolveFkChain(int i, Quaternion invSRot, Quaternion tRot)
        {
            var bones = fkBones[i];
            if (bones.Length == 0)
                return;

            int[] joints = fkChains[i].joints;
            for (int k = 0; k < bones.Length; k++)
            {
                int seg = fkSegIndex[i][k];
                Vector3 segDir = SmalPos(joints[seg + 1]) - SmalPos(joints[seg]);
                if (segDir.sqrMagnitude < Eps)
                    continue;
                Vector3 curDir = (invSRot * segDir).normalized;
                Quaternion swing = Quaternion.FromToRotation(fkRefDirT[i][k], curDir);
                bones[k].rotation = tRot * swing * fkBindRel[i][k];
            }
            gizmoTargets[legs.Length + chains.Length + i] = bones[bones.Length - 1].position;
        }

        public Transform BodyRoot => targetRoot != null ? targetRoot : transform;

        public bool TryGetBodyFrame(out Vector3 origin, out Vector3 forward)
        {
            if (tLsh == null || tRsh == null || tLhip == null || tRhip == null)
            {
                origin = default;
                forward = default;
                return false;
            }
            Vector3 shMid = (tLsh.position + tRsh.position) * 0.5f;
            Vector3 hipMid = (tLhip.position + tRhip.position) * 0.5f;
            origin = (shMid + hipMid) * 0.5f;
            forward = shMid - hipMid;
            return forward.sqrMagnitude > Eps;
        }

        void PlaceRoot(Quaternion sRot, Vector3 sOrigin)
        {
            TorsoFrame(tLsh.position, tRsh.position, tLhip.position, tRhip.position,
                       out Quaternion tRot0, out _, out _);
            Quaternion delta = RootAxisMask.Rotation(
                sRot * Quaternion.Inverse(tRot0), globalRotationAxes);
            targetRoot.rotation = delta * targetRoot.rotation;

            TorsoFrame(tLsh.position, tRsh.position, tLhip.position, tRhip.position,
                       out _, out Vector3 tOrigin1, out _);
            targetRoot.position += RootAxisMask.Vector(sOrigin - tOrigin1, globalTranslationAxes);
        }

        static Vector3 MapOffset(Vector3 point, Vector3 root, Quaternion sRot,
                                 Quaternion tRot, Vector3 anchor, float scale)
        {
            Vector3 local = Quaternion.Inverse(sRot) * (point - root);
            return anchor + tRot * (local * scale);
        }

        static void LockRoll(Transform bone, Quaternion bindLocal, Vector3 axisLocal, float maxDeg)
        {
            Quaternion delta = Quaternion.Inverse(bindLocal) * bone.localRotation;
            Vector3 proj = Vector3.Project(new Vector3(delta.x, delta.y, delta.z), axisLocal);
            float norm2 = proj.sqrMagnitude + delta.w * delta.w;
            Quaternion twist;
            if (norm2 < Eps)
                twist = Quaternion.identity;
            else
            {
                float inv = 1f / Mathf.Sqrt(norm2);
                twist = new Quaternion(proj.x * inv, proj.y * inv, proj.z * inv, delta.w * inv);
            }

            Quaternion swing = delta * Quaternion.Inverse(twist);
            twist = Quaternion.RotateTowards(Quaternion.identity, twist, maxDeg);
            bone.localRotation = bindLocal * swing * twist;
        }

        void LegDirs(RetargetLeg leg, Quaternion sRot, Quaternion tRot,
                     out Vector3 dirU, out Vector3 dirL, out Vector3 normal)
        {
            Quaternion m = tRot * Quaternion.Inverse(sRot);
            dirU = m * (SmalPos(leg.poleJoint) - SmalPos(leg.rootJoint));
            dirL = m * (SmalPos(leg.targetJoint) - SmalPos(leg.poleJoint));
            normal = Vector3.Cross(dirU, dirL);
            if (normal.sqrMagnitude < Eps)
                normal = tRot * Vector3.up;
        }

        static Vector3 StableNormal(ref Vector3 prev, Vector3 normal)
        {
            if (normal.sqrMagnitude <= Eps)
                return prev != Vector3.zero ? prev : Vector3.up;
            normal.Normalize();
            if (prev != Vector3.zero && Vector3.Dot(normal, prev) < 0f)
                normal = -normal;
            prev = normal;
            return normal;
        }

        static void TorsoFrame(Vector3 lsh, Vector3 rsh, Vector3 lhip, Vector3 rhip,
                               out Quaternion rot, out Vector3 origin, out float len)
        {
            Vector3 shMid = (lsh + rsh) * 0.5f;
            Vector3 hipMid = (lhip + rhip) * 0.5f;
            origin = (shMid + hipMid) * 0.5f;

            Vector3 fwd = shMid - hipMid;
            Vector3 lat = rsh - lsh;
            Vector3 up = Vector3.Cross(fwd, lat);
            len = fwd.magnitude;
            if (fwd.sqrMagnitude < Eps || up.sqrMagnitude < Eps)
                throw new InvalidOperationException("SmalRetargeterDirect: degenerate torso frame");

            rot = Quaternion.LookRotation(fwd, up);
        }

        /// <summary>Reconstructed SMAL joint position, One-Euro-smoothed and cached once
        /// per LateUpdate so every reader in a frame sees the same filtered value and the
        /// filter advances exactly once per joint per frame.</summary>
        Vector3 SmalPos(int joint)
        {
            if (smalPosValid != null && smalPosValid[joint])
                return smalPosFrame[joint];

            Vector3 val = smalWorldPos[joint];
            if (smoothSource && smalFilters != null && smalFilters[joint] != null)
                val = smalFilters[joint].Filter(val, Time.deltaTime);
            if (smalPosValid != null)
            {
                smalPosFrame[joint] = val;
                smalPosValid[joint] = true;
            }
            return val;
        }

        void OnDrawGizmos()
        {
            if (!drawGizmos)
                return;
            if (gizmoTargets != null)
            {
                Gizmos.color = Color.green;
                foreach (var g in gizmoTargets)
                    Gizmos.DrawSphere(g, gizmoRadius);
            }
            if (gizmoPoles != null)
            {
                Gizmos.color = Color.yellow;
                foreach (var p in gizmoPoles)
                    Gizmos.DrawSphere(p, gizmoRadius * 0.6f);
            }
        }
    }
}
