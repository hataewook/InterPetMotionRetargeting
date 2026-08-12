using System.Collections.Generic;
using UnityEngine;

namespace PetDemo
{
    /// <summary>How the legs are posed by <see cref="SmalRetargeter"/>.</summary>
    public enum RetargetMode
    {
        IK,           // bend legs so the paws reach the mapped SMAL foot positions (feet planted)
        FK,           // transfer the SMAL leg-segment directions onto the bones (feet may float)
    }

    /// <summary>
    /// Retargets the live SMAL motion (driven by a <see cref="SmalMotionPlayer"/>
    /// on a separate source rig) onto a mapped target rig, preserving the
    /// target's own proportions.
    ///
    /// A torso frame (forward = hips->shoulders, lateral = left->right shoulder,
    /// up = forward x lateral) is built each frame on both rigs from four SMAL
    /// joints and their mapped target bones; it carries the body orientation.
    /// Each limb/trunk goal is then transferred per part: the SMAL end-point offset
    /// from the part's root joint is expressed in the SMAL torso frame, divided by
    /// that part's current SMAL length, scaled by the target part's rest length,
    /// and placed from the target part's root bone in the target torso frame.
    /// Because each goal is normalized by the part's own length, it can never reach
    /// past the target's own bones (no enlargement) and the source rig's per-frame
    /// global scale (s_world) cancels out. Legs solve with analytic two-bone IK
    /// (mapped knee/elbow as the pole, so bends never fold/twist the wrong way), with
    /// an optional paw/toe bone driven by FK on top so the toe articulates without
    /// disturbing the planted paw; a
    /// trunk goal can also be reached with CCD. The spine, neck and tail are
    /// transferred per segment (FK): each bone follows the direction of its mapped
    /// SMAL segment through the body frame, so the whole curve of the back/tail is
    /// reproduced, any length difference is absorbed for free, and no twist is
    /// introduced. Spine FK runs before the legs (it moves the shoulders, so the
    /// torso frame is re-measured afterwards). The jaw and ears (SMAL 32/33/34) are
    /// copied by local rotation only, with a per-axis remap for differing bone frames.
    /// All solves are rotation only.
    ///
    /// Runs in LateUpdate so it reads the SMAL rig after SmalMotionPlayer has posed
    /// it for the frame. Attach to the target root and assign the SMAL source rig.
    /// </summary>
    public class SmalRetargeter : MonoBehaviour, IBodyFrameProvider
    {
        [Header("Source")]
        [Tooltip("Root of the SMAL rig driven by SmalMotionPlayer (contains the SMAL_joint bones).")]
        [SerializeField] Transform smalSource;
        [Tooltip("Bone name prefix on the source rig; index is two digits (e.g. SMAL_joint_09).")]
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
        [Tooltip("Copy a SMAL joint's local rotation (jaw 32, ears 33/34) onto a target " +
                 "bone. Rotation only; use the per-axis map to reorient it when the target " +
                 "bone's axes differ. Leave the target bone empty to skip.")]
        [SerializeField] RetargetCopyRotation[] copyRotations = SmalRetargetMap.DefaultCopyRotations();

        [Header("Global motion")]
        [Tooltip("Overlay the target body onto the SMAL body (follow its turn/translation). " +
                 "Leave off to keep the target planted and only play the articulation.")]
        [SerializeField] bool applyGlobalMotion;
        [Tooltip("Which world axes the body-overlay rotation may act on. The overlay " +
                 "matches the full torso frame (forward + up), so clear Z/X to stop a " +
                 "roll/pitch from tipping the whole body; keep Y for turning only.")]
        [SerializeField] RootAxis globalRotationAxes = RootAxis.All;
        [Tooltip("Which world axes the body-overlay translation may act on. Clear an " +
                 "axis to keep the body fixed on it (e.g. clear Y to stay on the ground).")]
        [SerializeField] RootAxis globalTranslationAxes = RootAxis.All;

        [Header("Smoothing (One-Euro on source joints)")]
        [Tooltip("Adaptively low-pass the SMAL source joint positions before retargeting: " +
                 "strong smoothing when slow, no lag when fast.")]
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

        // Source rig: SMAL_joint bones by index, and the player driving them (for the
        // neutral/rest pose used as the copy-rotation reference).
        Transform[] smalBones;
        SmalMotionPlayer smalPlayer;

        // Target rig: resolved legs and their rest lengths.
        Transform[] legUpper, legLower, legTip;
        float[] legLenTarget;
        Quaternion[] fkCalUpper, fkCalLower;   // FK rest calibration (bind-relative)
        Quaternion[] bindLocalUpper, bindLocalLower;   // leg bind local rotations (roll lock)
        Vector3[] rollAxisUpper, rollAxisLower;         // leg length axes in bone-local space
        // Optional paw/toe bone driven by FK on top of the leg IK (null = none).
        Transform[] legToe;
        Vector3[] toeRefDir;           // [leg] bind SMAL toe segment dir in the SMAL torso frame
        Quaternion[] toeBindRel;       // [leg] toe bind rotation relative to the target torso frame
        // Target rig: resolved trunk chains and their rest lengths.
        Transform[][] chainBones;
        Transform[] chainEffectors;
        float[] chainLenTarget;
        Transform tLsh, tRsh, tLhip, tRhip;
        // Target rig: resolved FK chains (spine / neck / tail), per-bone SMAL segment
        // mapping and bind refs.
        Transform[][] fkBones;
        int[][] fkSegIndex;            // [chain][bone] -> SMAL segment (joints[i]->joints[i+1])
        Vector3[][] fkRefDirT;         // [chain][bone] bind SMAL segment dir in the SMAL torso frame
        Quaternion[][] fkBindRel;      // [chain][bone] bind bone rotation relative to the target torso frame
        bool[] fkPreBody;              // [chain] true if it moves the shoulders/hips (solve before the legs)
        // Copy-rotation (jaw / ears): resolved target bone, bind local rotations and axis remap.
        Transform[] copyBone;
        Quaternion[] copySmalBind;     // [i] SMAL joint local rotation at frame 0
        Quaternion[] copyTargetBind;   // [i] target bone local rotation at bind
        Vector3[] copyRemapX, copyRemapY, copyRemapZ;   // [i] target-local axis per SMAL local axis

        // Bind cache for a deterministic solve each frame.
        Transform[] allBones;
        Quaternion[] bindLocalRot;
        Vector3[] bindLocalPos;
        Vector3 rootBindLocalPos;
        Quaternion rootBindLocalRot;

        Vector3[] gizmoTargets;        // end-point goals (green)
        Vector3[] gizmoPoles;          // leg pole hints (yellow)
        Vector3[] legBendAxis;         // [leg] persistent IK bend axis (pole-based, held near-straight)

        // Smoothing / contact: One-Euro filter per source joint (cached per LateUpdate),
        // stance lock per leg, and FK leg-plane normal continuity per leg.
        OneEuroFilterVector3[] smalFilters;
        Vector3[] smalPosFrame;
        bool[] smalPosValid;
        FootPlanter[] footPlanters;
        Vector3[] prevLegNormalSmal;

        void Awake()
        {
            if (targetRoot == null)
                targetRoot = transform;
            if (smalSource == null)
                throw new MissingReferenceException("SmalRetargeter: no SMAL source rig assigned");

            // A component saved before the FK-chain field existed deserializes it as
            // null, and a chain added in the Inspector starts with empty joints; fill
            // the joints from the chain's name (spine/neck/tail) so assigning the bone
            // chain is enough.
            if (fkChains == null)
                fkChains = System.Array.Empty<RetargetFkChain>();
            for (int i = 0; i < fkChains.Length; i++)
                if ((fkChains[i].joints == null || fkChains[i].joints.Length < 2)
                    && fkChains[i].bones != null && fkChains[i].bones.Length > 0)
                    fkChains[i].joints = SmalRetargetMap.DefaultJoints(fkChains[i].name);

            // A toe bone with no explicit toe joint points at the paw's SMAL child
            // (targetJoint + 1), which is the toe on the standard SMAL leg chains.
            for (int i = 0; i < legs.Length; i++)
                if (legs[i].toeBone != null && legs[i].toeJoint <= 0)
                    legs[i].toeJoint = legs[i].targetJoint + 1;

            // The player on the source rig provides the neutral (rest) pose that the
            // copy-rotation entries measure their delta against.
            smalPlayer = smalSource.GetComponentInParent<SmalMotionPlayer>();
            if (smalPlayer == null)
                smalPlayer = smalSource.GetComponentInChildren<SmalMotionPlayer>();

            // New field on components saved before it existed deserializes as null.
            if (copyRotations == null)
                copyRotations = System.Array.Empty<RetargetCopyRotation>();
            // A newly-added Inspector element (or data saved before the weight field)
            // starts at 0; treat that as 1 so it copies 1:1. Set 0 in Play to freeze.
            for (int i = 0; i < copyRotations.Length; i++)
                if (copyRotations[i].weight <= 0f)
                    copyRotations[i].weight = 1f;

            ResolveSourceBones();
            ResolveTargetRig();
            CacheBind();

            // One-Euro filter per resolved source joint; positions are cached per
            // LateUpdate so SmalPos filters each joint exactly once a frame.
            smalFilters = new OneEuroFilterVector3[smalBones.Length];
            smalPosFrame = new Vector3[smalBones.Length];
            smalPosValid = new bool[smalBones.Length];
            for (int j = 0; j < smalBones.Length; j++)
                if (smalBones[j] != null)
                    smalFilters[j] = new OneEuroFilterVector3(sourceMinCutoff, sourceBeta);

            foreach (var smr in targetRoot.GetComponentsInChildren<SkinnedMeshRenderer>())
                smr.updateWhenOffscreen = true;
            foreach (var animator in targetRoot.GetComponentsInChildren<Animator>())
                animator.enabled = false;
        }

        void Start()
        {
            // Measure each part's rest length at bind (constant under a rotation-only
            // solve). Goals are normalized to these so the target keeps its size.
            legLenTarget = new float[legs.Length];
            for (int i = 0; i < legs.Length; i++)
                legLenTarget[i] = (legLower[i].position - legUpper[i].position).magnitude
                                + (legTip[i].position - legLower[i].position).magnitude;

            chainLenTarget = new float[chains.Length];
            for (int i = 0; i < chains.Length; i++)
                chainLenTarget[i] = (chainEffectors[i].position - chainBones[i][0].position).magnitude;

            // FK calibration: the rotation that turns the frame-0 SMAL leg-segment
            // orientation into each bone's bind rotation, so FK reproduces the bind
            // pose at frame 0 and adds only the captured articulation afterwards.
            fkCalUpper = new Quaternion[legs.Length];
            fkCalLower = new Quaternion[legs.Length];
            footPlanters = new FootPlanter[legs.Length];
            prevLegNormalSmal = new Vector3[legs.Length];
            for (int i = 0; i < legs.Length; i++)
                footPlanters[i] = new FootPlanter();
            TorsoFrame(SmalPos(SmalRetargetMap.LeftShoulder), SmalPos(SmalRetargetMap.RightShoulder),
                       SmalPos(SmalRetargetMap.LeftHip), SmalPos(SmalRetargetMap.RightHip),
                       out Quaternion sRot0, out _, out _);
            TorsoFrame(tLsh.position, tRsh.position, tLhip.position, tRhip.position,
                       out Quaternion tRot0, out _, out _);
            for (int i = 0; i < legs.Length; i++)
            {
                LegDirs(legs[i], sRot0, tRot0, out Vector3 dU, out Vector3 dL, out Vector3 n);
                fkCalUpper[i] = Quaternion.Inverse(Quaternion.LookRotation(dU, n)) * legUpper[i].rotation;
                fkCalLower[i] = Quaternion.Inverse(Quaternion.LookRotation(dL, n)) * legLower[i].rotation;
                prevLegNormalSmal[i] = n.normalized;   // seed FK continuity at calibration sign
            }

            // Roll-lock reference: each leg bone's bind local rotation and its length
            // axis in bone-local space (direction to the next joint). Twisting a bone
            // about this axis leaves the next joint in place, so locking the twist
            // keeps the IK foot position while removing unwanted roll.
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

            // Toe calibration (per leg with a toe bone): the same segment-FK reference
            // as the spine/tail chains, for the single SMAL segment paw -> toe.
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
                        throw new System.InvalidOperationException(
                            $"SmalRetargeter: leg '{legs[i].name}' toe segment is degenerate at bind");
                    toeRefDir[i] = (invSRot0 * segDir).normalized;
                    toeBindRel[i] = invTRot0 * legToe[i].rotation;
                }
            }

            // FK-chain calibration (spine / neck / tail): for each target bone, record
            // the SMAL segment it tracks (proportional mapping when the counts differ),
            // that segment's bind direction in the SMAL torso frame, and the bone's bind
            // rotation relative to the target torso frame. Each frame we apply only the
            // swing that turns the bind segment direction into the current one, so the
            // bone follows the chain's bend while keeping its bind roll and its own length.
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
                        throw new System.InvalidOperationException(
                            $"SmalRetargeter: FK chain '{fkChains[i].name}' segment {seg} is degenerate at bind");
                    fkSegIndex[i][k] = seg;
                    fkRefDirT[i][k] = (invSRot0 * segDir).normalized;
                    fkBindRel[i][k] = invTRot0 * bones[k].rotation;
                }
            }

            // Copy-rotation calibration (jaw / ears): record the SMAL joint's NEUTRAL
            // (rest) local rotation and the target bone's bind local rotation, plus the
            // target axis each SMAL local axis maps to. Each frame we take the joint's
            // rotation relative to that neutral, remap its axis, and add it to the target
            // bind — so the copy is the change from rest, not a raw rotation. The rest
            // pose comes from the player (captured before frame 0); with no player we
            // fall back to frame 0 as the neutral.
            copySmalBind = new Quaternion[copyRotations.Length];
            copyTargetBind = new Quaternion[copyRotations.Length];
            copyRemapX = new Vector3[copyRotations.Length];
            copyRemapY = new Vector3[copyRotations.Length];
            copyRemapZ = new Vector3[copyRotations.Length];
            bool anyCopyMissingNeutral = false;
            for (int i = 0; i < copyRotations.Length; i++)
            {
                if (copyBone[i] == null)
                    continue;
                var cr = copyRotations[i];
                if (smalPlayer == null || !smalPlayer.TryGetRestLocalRotation(cr.joint, out copySmalBind[i]))
                {
                    copySmalBind[i] = smalBones[cr.joint].localRotation;   // fallback: frame 0
                    anyCopyMissingNeutral = true;
                }
                copyTargetBind[i] = copyBone[i].localRotation;
                copyRemapX[i] = RetargetCopyRotation.AxisVec(cr.mapX);
                copyRemapY[i] = RetargetCopyRotation.AxisVec(cr.mapY);
                copyRemapZ[i] = RetargetCopyRotation.AxisVec(cr.mapZ);
            }
            if (anyCopyMissingNeutral)
                Debug.LogWarning("SmalRetargeter: no SmalMotionPlayer on the source rig; " +
                    "copy-rotation uses frame 0 as the neutral instead of the true rest pose.");

            if (logCalibration)
                Debug.Log($"SmalRetargeter: rest leg lengths=[{string.Join(", ", legLenTarget)}]");
        }

        void ResolveSourceBones()
        {
            var byName = new Dictionary<string, Transform>();
            foreach (Transform t in smalSource.GetComponentsInChildren<Transform>(true))
                byName[t.name] = t;

            int maxJoint = 0;
            foreach (int j in RequiredJoints())
                maxJoint = Mathf.Max(maxJoint, j);

            smalBones = new Transform[maxJoint + 1];
            foreach (int j in RequiredJoints())
            {
                string wanted = $"{boneNamePrefix}{j:00}";
                if (!byName.TryGetValue(wanted, out smalBones[j]))
                    throw new MissingReferenceException(
                        $"SmalRetargeter: source bone '{wanted}' not found under {smalSource.name}");
            }
        }

        IEnumerable<int> RequiredJoints()
        {
            yield return SmalRetargetMap.LeftShoulder;
            yield return SmalRetargetMap.RightShoulder;
            yield return SmalRetargetMap.LeftHip;
            yield return SmalRetargetMap.RightHip;
            foreach (var leg in legs)
            {
                yield return leg.rootJoint;
                yield return leg.targetJoint;
                yield return leg.poleJoint;
                if (leg.toeBone != null)
                    yield return leg.toeJoint;
            }
            foreach (var chain in chains)
            {
                yield return chain.rootJoint;
                yield return chain.endJoint;
            }
            foreach (var fk in fkChains)
            {
                if (fk.bones == null || fk.bones.Length == 0 || fk.joints == null)
                    continue;
                foreach (int j in fk.joints)
                    yield return j;
            }
            foreach (var cr in copyRotations)
                if (cr.targetBone != null)
                    yield return cr.joint;
        }

        void ResolveTargetRig()
        {
            Transform Require(Transform bone, string mapping)
            {
                if (bone == null)
                    throw new MissingReferenceException(
                        $"SmalRetargeter: target mapping '{mapping}' has no bone assigned");
                if (bone != targetRoot && !bone.IsChildOf(targetRoot))
                    throw new System.InvalidOperationException(
                        $"SmalRetargeter: mapped bone '{bone.name}' is not under " +
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
                chainEffectors[i] = Require(
                    chains[i].effectorBone, $"{chains[i].name} Effector");
                chainBones[i] = BuildChain(baseBone, chainEffectors[i], chains[i].name);
            }

            fkBones = new Transform[fkChains.Length][];
            fkPreBody = new bool[fkChains.Length];
            for (int i = 0; i < fkChains.Length; i++)
            {
                var fk = fkChains[i];
                if (fk.bones == null || fk.bones.Length == 0)
                {
                    fkBones[i] = System.Array.Empty<Transform>();
                    continue;
                }
                if (fk.joints == null || fk.joints.Length < 2)
                    throw new MissingReferenceException(
                        $"SmalRetargeter: FK chain '{fk.name}' needs at least two SMAL joints " +
                        "(name it spine/neck/tail to auto-fill, or set them explicitly)");

                var bones = new Transform[fk.bones.Length];
                for (int k = 0; k < bones.Length; k++)
                {
                    bones[k] = Require(fk.bones[k], $"{fk.name} bone {k}");
                    if (k > 0)
                        RequireDescendant(bones[k - 1], bones[k], fk.name);
                }
                fkBones[i] = bones;
                // A chain that is an ancestor of any torso-frame bone (the spine carries
                // the shoulders) must be posed before the legs read the torso frame.
                fkPreBody[i] = MovesTorsoFrame(bones);
            }

            copyBone = new Transform[copyRotations.Length];
            for (int i = 0; i < copyRotations.Length; i++)
                if (copyRotations[i].targetBone != null)
                    copyBone[i] = Require(copyRotations[i].targetBone, $"{copyRotations[i].name} target");
        }

        /// <summary>True if any of the four torso-frame bones is one of these bones or
        /// a descendant of one — i.e. posing this chain moves the shoulders/hips, so it
        /// must be solved before the torso frame is measured for the legs.</summary>
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
                $"SmalRetargeter: leg '{name}' bones are not a parent->child chain " +
                $"({ancestor.name} -> {descendant.name})");
        }

        /// <summary>Rotatable bones from <paramref name="baseBone"/> down to the
        /// effector's parent, base first. Walks the actual hierarchy so a broken
        /// mapping fails loudly instead of solving a wrong chain.</summary>
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
                    $"SmalRetargeter: chain '{name}' is not a contiguous chain from " +
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

            // Invalidate the per-frame source-position cache so each joint is filtered
            // once this frame.
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

            // Pre-body FK chains (the spine carries the shoulders) are posed first so
            // the torso frame the legs read reflects the bent back.
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
                    LegDirs(leg, sRot, tRot, out Vector3 dU, out Vector3 dL, out Vector3 n);
                    n = StableNormal(ref prevLegNormalSmal[i], n);
                    legUpper[i].rotation = Quaternion.LookRotation(dU, n) * fkCalUpper[i];
                    legLower[i].rotation = Quaternion.LookRotation(dL, n) * fkCalLower[i];

                    // Plant the FK foot during stance with a light IK correction.
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
                    gizmoTargets[i] = legTip[i].position;
                    gizmoPoles[i] = legLower[i].position;
                    continue;
                }

                Vector3 sRoot = SmalPos(leg.rootJoint);
                Vector3 sPole = SmalPos(leg.poleJoint);
                Vector3 sFoot = SmalPos(leg.targetJoint);
                float srcLen = (sPole - sRoot).magnitude + (sFoot - sPole).magnitude;
                if (srcLen < Eps)
                    throw new System.InvalidOperationException($"SmalRetargeter: leg '{leg.name}' collapsed");

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
                gizmoTargets[i] = target;
                gizmoPoles[i] = pole;
            }

            // Toe FK: after the IK (or FK) has posed each leg, orient the optional toe
            // bone to follow the SMAL paw->toe direction. The toe is a leaf, so setting
            // its world rotation does not disturb the planted paw.
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

            for (int i = 0; i < chains.Length; i++)
            {
                var chain = chains[i];
                Vector3 sBase = SmalPos(chain.rootJoint);
                Vector3 sEnd = SmalPos(chain.endJoint);
                float srcLen = (sEnd - sBase).magnitude;
                if (srcLen < Eps)
                    throw new System.InvalidOperationException($"SmalRetargeter: chain '{chain.name}' collapsed");

                float chainScale = chainLenTarget[i] / srcLen;
                Vector3 anchor = chainBones[i][0].position;
                Vector3 target = MapOffset(sEnd, sBase, sRot, tRot, anchor, chainScale);
                ChainIK.Solve(chainBones[i], chainEffectors[i], target, ikIterations);
                gizmoTargets[legs.Length + i] = target;
            }

            // Post-body FK chains (neck / tail) use the re-measured torso frame.
            for (int i = 0; i < fkChains.Length; i++)
                if (!fkPreBody[i])
                    SolveFkChain(i, invSRot, tRot);

            // Copy-rotation (jaw / ears): apply the SMAL joint's local delta with the
            // per-axis remap. Local rotation, so it is independent of the head's pose.
            for (int i = 0; i < copyRotations.Length; i++)
                SolveCopyRotation(i);
        }

        /// <summary>Copy one SMAL joint's local articulation onto its target bone: take
        /// the joint's rotation relative to its neutral (rest) pose as an axis-angle,
        /// remap the axis into the target bone's frame (per-axis signed map), scale the
        /// angle by the entry's weight, and add that to the target bind. Because the
        /// reference is the rest pose, this transfers only the change from neutral.
        /// Weight is read live so suppression can be tuned in Play.</summary>
        void SolveCopyRotation(int i)
        {
            if (copyBone[i] == null)
                return;

            Quaternion deltaS = Quaternion.Inverse(copySmalBind[i])
                                * smalBones[copyRotations[i].joint].localRotation;
            deltaS.ToAngleAxis(out float angle, out Vector3 axisS);
            if (angle > 180f)          // shortest arc, so the weight scales the real angle
                angle -= 360f;

            Vector3 axisT = axisS.x * copyRemapX[i] + axisS.y * copyRemapY[i] + axisS.z * copyRemapZ[i];
            Quaternion deltaT = Quaternion.AngleAxis(angle * copyRotations[i].weight, axisT);
            copyBone[i].localRotation = copyTargetBind[i] * deltaT;
        }

        /// <summary>Pose one FK chain: rotate each bone so it follows its mapped SMAL
        /// segment direction, carried into the target torso frame. Base first, so a
        /// parent's update never disturbs a child written afterwards (world rotations
        /// are set absolutely).</summary>
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

        /// <summary>The rig root this component places for root motion (when
        /// Apply Global Motion is on).</summary>
        public Transform BodyRoot => targetRoot != null ? targetRoot : transform;

        /// <summary>Current body frame from the posed torso: centre and forward
        /// (hips -> shoulders) of the target torso frame. Used by
        /// <see cref="RootMotionAnchor"/> to re-base the root motion.</summary>
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

        /// <summary>Overlay the target body onto the SMAL body: turn the root so
        /// the torso frames align, then slide it so the torso origins coincide.</summary>
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

        /// <summary>Take the source offset (point - root), express it in the SMAL
        /// torso frame, scale it by the part's length ratio, then place it from the
        /// target anchor in the target torso frame.</summary>
        static Vector3 MapOffset(Vector3 point, Vector3 root, Quaternion sRot,
                                 Quaternion tRot, Vector3 anchor, float scale)
        {
            Vector3 local = Quaternion.Inverse(sRot) * (point - root);
            return anchor + tRot * (local * scale);
        }

        /// <summary>
        /// Clamp a bone's twist (roll about its own length axis) to
        /// <paramref name="maxDeg"/> degrees from the rest roll, keeping the bend
        /// (swing). Swing-twist decomposition of the bind-relative local rotation
        /// about <paramref name="axisLocal"/>; because the axis points at the next
        /// joint, this does not move it. <paramref name="maxDeg"/> = 0 fully locks
        /// the roll.
        /// </summary>
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

        /// <summary>SMAL leg-segment directions (root->pole, pole->foot) and the
        /// leg-plane normal, carried from the SMAL torso frame into the target
        /// torso frame. Used by FK to orient the upper and lower leg bones.</summary>
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

        /// <summary>Keep an FK leg-plane normal continuous frame to frame. When the leg
        /// straightens the segments become collinear and the raw normal collapses or
        /// flips, snapping the LookRotation roll; carry the previous normal through the
        /// degenerate span and keep the sign consistent with calibration.</summary>
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

        /// <summary>Torso frame: forward = hips->shoulders, up = forward x lateral
        /// (lateral = left->right shoulder), origin = body centre, len = forward
        /// length.</summary>
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
                throw new System.InvalidOperationException("SmalRetargeter: degenerate torso frame");

            rot = Quaternion.LookRotation(fwd, up);
        }

        /// <summary>Source joint position, One-Euro-smoothed and cached once per
        /// LateUpdate so every reader in a frame sees the same filtered value and the
        /// filter advances exactly once per joint per frame.</summary>
        Vector3 SmalPos(int joint)
        {
            if (smalPosValid != null && smalPosValid[joint])
                return smalPosFrame[joint];

            Vector3 val = smalBones[joint].position;
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
