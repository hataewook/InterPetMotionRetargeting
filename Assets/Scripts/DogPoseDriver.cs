using System;
using System.Collections.Generic;
using UnityEngine;

namespace PetDemo
{
    public enum LegMode
    {
        IK,
        FK,
    }

    /// <summary>
    /// Drives any mapped dog rig from an InterPet4D MMPose-20 clip. The mapping
    /// is indexed directly by <see cref="DogKeypoint"/>, so the same component
    /// supports Dog_rejoint, P_GermanShepherd, and custom rigs.
    /// </summary>
    [DefaultExecutionOrder(100)]
    public class DogPoseDriver : MonoBehaviour, IBodyFrameProvider
    {
        [Header("Clips (switch by changing Clip Index at runtime)")]
        [SerializeField] TextAsset[] clips;
        [SerializeField] int clipIndex;

        [Header("Rig")]
        [Tooltip("Root containing the target skeleton. Defaults to this transform.")]
        [SerializeField] protected Transform dogRoot;
        [SerializeField] DogRigPreset mappingPreset = DogRigPreset.DogRejoint;
        [SerializeField] Transform[] jointBones =
            new Transform[DogJoints.Count];

        [Header("Head")]
        [Tooltip("Aim the mapped Throat bone toward Head Aim Keypoint's mapped bone.")]
        [SerializeField] bool aimHead = true;
        [SerializeField] DogKeypoint headAimKeypoint = DogKeypoint.L_Eye;

        [Header("Leg solving")]
        [Tooltip("IK follows paw positions. FK transfers captured segment directions.")]
        [SerializeField] LegMode legMode = LegMode.IK;

        [Header("Hip / femur (option 1: drive body->knee segment)")]
        [Tooltip("Aim each leg's femur bone (parent of the mapped Knee bone) along the " +
                 "captured anchor->Knee direction, so the largest, most-mobile leg " +
                 "segment is articulated instead of riding rigidly with the body. " +
                 "Disable if the rig has no dedicated per-leg thigh/upper-arm bone.")]
        [SerializeField] bool driveHip = true;
        [SerializeField, Range(0f, 1f)] float hipAimWeight = 1f;

        [Header("Solver weights")]
        [SerializeField, Range(0f, 1f)] float bodyPositionWeight = 1f;
        [SerializeField, Range(0f, 1f)] float bodyRotationWeight = 1f;
        [SerializeField, Range(0f, 1f)] float headAimWeight = 1f;
        [SerializeField, Range(0f, 1f)] float leftFrontLegWeight = 1f;
        [SerializeField, Range(0f, 1f)] float rightFrontLegWeight = 1f;
        [SerializeField, Range(0f, 1f)] float leftBackLegWeight = 1f;
        [SerializeField, Range(0f, 1f)] float rightBackLegWeight = 1f;
        [SerializeField, Range(0f, 1f)] float leftFrontPoleWeight = 1f;
        [SerializeField, Range(0f, 1f)] float rightFrontPoleWeight = 1f;
        [SerializeField, Range(0f, 1f)] float leftBackPoleWeight = 1f;
        [SerializeField, Range(0f, 1f)] float rightBackPoleWeight = 1f;

        [Header("IK leg roll")]
        [SerializeField] bool limitIkLegRoll = true;
        [SerializeField, Range(0f, 90f)] float maxIkLegRollDegrees = 15f;

        [Header("IK soft limit")]
        [Tooltip("Ease the last fraction of leg extension so the knee approaches full " +
                 "straight smoothly instead of snapping. 0 = off.")]
        [SerializeField, Range(0f, 0.2f)] float ikSoftZone = 0.03f;

        [Header("Smoothing (One-Euro on keypoints)")]
        [Tooltip("Adaptively low-pass the captured keypoints: strong smoothing when " +
                 "slow, no lag when fast. Removes per-frame tremor.")]
        [SerializeField] bool smoothKeypoints = true;
        [Tooltip("Base cutoff (Hz). Lower = smoother but laggier at rest.")]
        [SerializeField] float smoothingMinCutoff = 1.2f;
        [Tooltip("Speed coefficient. Higher = follows fast motion with less lag.")]
        [SerializeField] float smoothingBeta = 0.03f;

        [Header("Foot contact lock (anti-skate)")]
        [Tooltip("Freeze a paw in world space while it is planted so it stops sliding. " +
                 "In FK mode this adds a light IK correction only during stance.")]
        [SerializeField] bool lockPlantedFeet = true;
        [Tooltip("Enter stance when the paw's capture-space speed drops below this (m/s).")]
        [SerializeField] float footLockSpeed = 0.05f;
        [Tooltip("Leave stance when the paw's capture-space speed rises above this (m/s).")]
        [SerializeField] float footUnlockSpeed = 0.15f;
        [Tooltip("Ease time for a plant / release, seconds.")]
        [SerializeField] float footLockAttack = 0.08f;

        [Header("Capture -> Unity axes")]
        [Tooltip("Capture is Z-up; Unity is Y-up. Swap Y/Z to convert.")]
        [SerializeField] bool swapYZ = true;
        [Tooltip("Flip an axis if facing or left/right come out wrong.")]
        [SerializeField] Vector3 axisSign = Vector3.one;
        [Tooltip("Optional transform used to place/rotate the capture. Leave unscaled.")]
        [SerializeField] Transform captureOrigin;

        [Header("Scale")]
        [SerializeField] bool autoScaleDogToCapture = true;
        [SerializeField] float scaleMultiplier = 1f;

        [Header("Playback")]
        [Tooltip("0 = use the clip's own fps.")]
        [SerializeField] float fpsOverride;
        [SerializeField] bool loop = true;

        [Header("Root")]
        [Tooltip("Turn the body to face along captured TailBase -> Throat.")]
        [SerializeField] bool alignRootToTrunk = true;
        [Tooltip("Which world axes the trunk-align rotation may act on. Clear an axis " +
                 "to stop that rotation from reaching the body (e.g. clear Z to kill a " +
                 "roll about the forward axis that tips the whole body).")]
        [SerializeField] RootAxis alignRotationAxes = RootAxis.All;
        [Tooltip("Which world axes the root follows the captured body centre on. Clear " +
                 "an axis to keep the body fixed on it (e.g. clear Y to stay planted on " +
                 "the ground).")]
        [SerializeField] RootAxis alignTranslationAxes = RootAxis.All;

        [Header("Confidence")]
        [SerializeField] bool holdLastValid;
        [SerializeField, Range(0f, 1f)] float confidenceThreshold = 0.1f;

        [Header("Debug gizmos (Play mode)")]
        [SerializeField] bool drawKeypointGizmos = true;
        [SerializeField] bool drawJointLabels;
        [SerializeField] float gizmoJointRadius = 0.012f;

        static readonly DogKeypoint[,] LegKeypoints =
        {
            { DogKeypoint.L_F_Knee, DogKeypoint.L_F_Elbow, DogKeypoint.L_F_Paw },
            { DogKeypoint.R_F_Knee, DogKeypoint.R_F_Elbow, DogKeypoint.R_F_Paw },
            { DogKeypoint.L_B_Knee, DogKeypoint.L_B_Elbow, DogKeypoint.L_B_Paw },
            { DogKeypoint.R_B_Knee, DogKeypoint.R_B_Elbow, DogKeypoint.R_B_Paw },
        };

        protected struct ResolvedLeg
        {
            public Transform femur;
            public Transform upper;
            public Transform lower;
            public Transform end;
            public DogKeypoint hipAnchor;
            public DogKeypoint kneeJoint;
            public DogKeypoint poleJoint;
            public DogKeypoint targetJoint;
        }

        Transform tailBone;
        Transform throatBone;
        Transform headAimBone;
        protected ResolvedLeg[] legs;

        Transform[] allBones;
        Quaternion[] bindLocalRot;
        Vector3[] bindLocalPos;
        Vector3 rootBindLocalPos;
        Quaternion rootBindLocalRot;
        protected Vector3 rootBindLocalScale;
        Quaternion[] bindLocalUpper;
        Quaternion[] bindLocalLower;
        Vector3[] rollAxisUpper;
        Vector3[] rollAxisLower;

        [NonSerialized] DogPoseClip clip;
        [NonSerialized] int loadedIndex = -1;
        float time;
        readonly Vector3[] lastValidWorld = new Vector3[DogJoints.Count];
        Quaternion[] legCalUpper;
        Quaternion[] legCalLower;
        Vector3[] gizmoKeypoints;
        int gizmoFrame;

        OneEuroFilterVector3[] keypointFilters;   // [joint] temporal smoothing
        FootPlanter[] footPlanters;                // [leg] stance lock
        Vector3[] prevLegNormal;                   // [leg] FK leg-plane normal continuity

        protected void Reset()
        {
            dogRoot = transform;
            EnsureMappingSize();
        }

        protected void OnValidate()
        {
            EnsureMappingSize();
        }

        protected void Awake()
        {
            if (dogRoot == null)
                dogRoot = transform;

            if (MappingIsEmpty())
                ApplySelectedMappingPreset();
            else
                ResolveMapping();
            ValidateNoCompetingAnimation();
            CacheBind();
            InitSmoothing();
            LoadClip(clipIndex);
        }

        void InitSmoothing()
        {
            keypointFilters = new OneEuroFilterVector3[DogJoints.Count];
            for (int i = 0; i < keypointFilters.Length; i++)
                keypointFilters[i] =
                    new OneEuroFilterVector3(smoothingMinCutoff, smoothingBeta);
            footPlanters = new FootPlanter[legs.Length];
            for (int i = 0; i < footPlanters.Length; i++)
                footPlanters[i] = new FootPlanter();
            prevLegNormal = new Vector3[legs.Length];
        }

        /// <summary>Resolve the selected preset against the current rig hierarchy.</summary>
        [ContextMenu("Apply Selected Rig Mapping Preset")]
        public void ApplySelectedMappingPreset()
        {
            if (mappingPreset == DogRigPreset.Custom)
                throw new InvalidOperationException(
                    "Choose DogRejoint or GermanShepherd before applying a preset");
            Transform root = dogRoot != null ? dogRoot : transform;
            Dictionary<string, Transform> hierarchy = BuildHierarchy(root);
            string[] names = DogJoints.BoneNames(mappingPreset);
            jointBones = new Transform[DogJoints.Count];
            for (int index = 0; index < names.Length; index++)
            {
                string boneName = names[index];
                if (string.IsNullOrEmpty(boneName))
                    continue;
                if (!hierarchy.TryGetValue(boneName, out Transform bone))
                    throw new MissingReferenceException(
                        $"{mappingPreset}: bone '{boneName}' was not found under {root.name}");
                jointBones[index] = bone;
            }
            aimHead = mappingPreset == DogRigPreset.DogRejoint;
            headAimKeypoint = DogKeypoint.L_Eye;
            ResolveMapping();
        }

        [ContextMenu("Validate Joint Mapping")]
        public void ValidateMapping()
        {
            if (dogRoot == null)
                dogRoot = transform;
            ResolveMapping();
        }

        void EnsureMappingSize()
        {
            if (jointBones != null && jointBones.Length == DogJoints.Count)
                return;
            var resized = new Transform[DogJoints.Count];
            if (jointBones != null)
                Array.Copy(jointBones, resized, Math.Min(jointBones.Length, resized.Length));
            jointBones = resized;
        }

        bool MappingIsEmpty()
        {
            EnsureMappingSize();
            foreach (Transform bone in jointBones)
            {
                if (bone != null)
                    return false;
            }
            return true;
        }

        static Dictionary<string, Transform> BuildHierarchy(Transform root)
        {
            var result = new Dictionary<string, Transform>();
            foreach (Transform candidate in root.GetComponentsInChildren<Transform>(true))
            {
                if (result.ContainsKey(candidate.name))
                    throw new InvalidOperationException(
                        $"Rig '{root.name}' contains duplicate transform name '{candidate.name}'");
                result.Add(candidate.name, candidate);
            }
            return result;
        }

        void ResolveMapping()
        {
            EnsureMappingSize();
            tailBone = RequireMappedBone(DogKeypoint.TailBase);
            throatBone = RequireMappedBone(DogKeypoint.Throat);
            headAimBone = aimHead ? RequireMappedBone(headAimKeypoint) : null;

            legs = new ResolvedLeg[LegKeypoints.GetLength(0)];
            for (int index = 0; index < legs.Length; index++)
            {
                DogKeypoint knee = LegKeypoints[index, 0];
                DogKeypoint pole = LegKeypoints[index, 1];
                DogKeypoint target = LegKeypoints[index, 2];
                Transform upper = RequireMappedBone(knee);
                Transform lower = RequireMappedBone(pole);
                Transform end = RequireMappedBone(target);
                if (lower.parent != upper || end.parent != lower)
                    throw new InvalidOperationException(
                        $"Mapped leg must be contiguous: {upper.name} -> " +
                        $"{lower.name} -> {end.name}");
                legs[index] = new ResolvedLeg
                {
                    femur = upper.parent,
                    upper = upper,
                    lower = lower,
                    end = end,
                    // Legs 0,1 are front (attach at Throat); 2,3 are back (TailBase).
                    hipAnchor = index < 2 ? DogKeypoint.Throat : DogKeypoint.TailBase,
                    kneeJoint = knee,
                    poleJoint = pole,
                    targetJoint = target,
                };
            }
            ValidateFemurBones();
        }

        /// <summary>
        /// The hip step only applies when the mapped Knee bone is the shank (its
        /// pivot sits at the knee) and its parent is a dedicated per-leg femur, as
        /// in Dog_rejoint. Some rigs instead map Knee directly onto the femur/thigh
        /// bone (e.g. the Realistic Shepherd: Knee -> DEF-thigh.L), so the two-bone
        /// IK already drives the femur and the thigh hangs off the shared spine.
        /// There the hip step is both unnecessary and unsafe (it would drag the
        /// spine / other legs), so we detect that convention and disable hip drive
        /// for the whole rig rather than throwing.
        /// </summary>
        void ValidateFemurBones()
        {
            if (!driveHip)
                return;
            Transform root = dogRoot != null ? dogRoot : transform;
            string reason = null;
            for (int index = 0; index < legs.Length && reason == null; index++)
            {
                Transform femur = legs[index].femur;
                if (femur == null || femur == root || !femur.IsChildOf(root))
                {
                    reason = $"leg {index} Knee bone '{legs[index].upper.name}' has no " +
                             $"femur parent under '{root.name}'";
                    break;
                }
                for (int other = 0; other < legs.Length; other++)
                {
                    if (other == index)
                        continue;
                    if (legs[other].upper.IsChildOf(femur))
                    {
                        reason = $"femur '{femur.name}' of leg {index} is shared with " +
                                 $"leg {other} ('{legs[other].upper.name}')";
                        break;
                    }
                }
            }
            if (reason == null)
                return;
            Debug.LogWarning(
                $"DogPoseDriver on '{name}': Drive Hip skipped — {reason}. This rig's " +
                "mapped leg chain already starts at the femur (or lacks dedicated " +
                "per-leg thigh bones), so the hip step does not apply here.");
            for (int index = 0; index < legs.Length; index++)
                legs[index].femur = null;
        }

        Transform RequireMappedBone(DogKeypoint keypoint)
        {
            Transform bone = jointBones[(int)keypoint];
            if (bone == null)
                throw new MissingReferenceException(
                    $"DogPoseDriver: {keypoint} has no target bone mapping");
            Transform root = dogRoot != null ? dogRoot : transform;
            if (bone != root && !bone.IsChildOf(root))
                throw new InvalidOperationException(
                    $"Mapped bone '{bone.name}' is not under dog root '{root.name}'");
            return bone;
        }

        void ValidateNoCompetingAnimation()
        {
            foreach (Animator animator in dogRoot.GetComponentsInChildren<Animator>(true))
            {
                if (animator.enabled && animator.runtimeAnimatorController != null)
                    throw new InvalidOperationException(
                        $"Disable Animator '{animator.name}' before using DogPoseDriver; " +
                        "both components would write the same bones");
            }
            foreach (SmalMotionPlayer player in
                     dogRoot.GetComponentsInChildren<SmalMotionPlayer>(true))
            {
                if (player.enabled)
                    throw new InvalidOperationException(
                        $"Disable SmalMotionPlayer '{player.name}' before using DogPoseDriver");
            }
        }

        void CacheBind()
        {
            allBones = dogRoot.GetComponentsInChildren<Transform>(true);
            bindLocalRot = new Quaternion[allBones.Length];
            bindLocalPos = new Vector3[allBones.Length];
            for (int index = 0; index < allBones.Length; index++)
            {
                bindLocalRot[index] = allBones[index].localRotation;
                bindLocalPos[index] = allBones[index].localPosition;
            }
            rootBindLocalPos = dogRoot.localPosition;
            rootBindLocalRot = dogRoot.localRotation;
            rootBindLocalScale = dogRoot.localScale;

            bindLocalUpper = new Quaternion[legs.Length];
            bindLocalLower = new Quaternion[legs.Length];
            rollAxisUpper = new Vector3[legs.Length];
            rollAxisLower = new Vector3[legs.Length];
            for (int index = 0; index < legs.Length; index++)
            {
                ResolvedLeg leg = legs[index];
                bindLocalUpper[index] = leg.upper.localRotation;
                bindLocalLower[index] = leg.lower.localRotation;
                Vector3 upperDirection = leg.lower.position - leg.upper.position;
                Vector3 lowerDirection = leg.end.position - leg.lower.position;
                if (upperDirection.sqrMagnitude < 1e-8f ||
                    lowerDirection.sqrMagnitude < 1e-8f)
                    throw new InvalidOperationException(
                        $"DogPoseDriver: leg {index} has a zero-length bone");
                rollAxisUpper[index] =
                    (Quaternion.Inverse(leg.upper.rotation) * upperDirection).normalized;
                rollAxisLower[index] =
                    (Quaternion.Inverse(leg.lower.rotation) * lowerDirection).normalized;
            }
        }

        void RestoreBind()
        {
            for (int index = 0; index < allBones.Length; index++)
            {
                allBones[index].localRotation = bindLocalRot[index];
                allBones[index].localPosition = bindLocalPos[index];
            }
            dogRoot.localPosition = rootBindLocalPos;
            dogRoot.localRotation = rootBindLocalRot;
        }

        void LoadClip(int index)
        {
            if (clips == null || clips.Length == 0)
                throw new MissingReferenceException("DogPoseDriver: no clips assigned");
            index = Mathf.Clamp(index, 0, clips.Length - 1);
            if (clips[index] == null)
                throw new MissingReferenceException(
                    $"DogPoseDriver: clip slot {index} is empty");

            clip = DogPoseClip.FromJson(clips[index].text);
            loadedIndex = index;
            clipIndex = index;
            time = 0f;
            Array.Clear(lastValidWorld, 0, lastValidWorld.Length);
            if (keypointFilters != null)
                foreach (var f in keypointFilters) f.Reset();
            if (footPlanters != null)
                foreach (var p in footPlanters) p.Reset();
            if (prevLegNormal != null)
                Array.Clear(prevLegNormal, 0, prevLegNormal.Length);
            RestoreBind();
            ApplyScale();
            OnClipLoaded();
            CalibrateLegs();
        }

        protected virtual void OnClipLoaded()
        {
        }

        void CalibrateLegs()
        {
            legCalUpper = new Quaternion[legs.Length];
            legCalLower = new Quaternion[legs.Length];
            var keypoints = new Vector3[DogJoints.Count];
            for (int joint = 0; joint < DogJoints.Count; joint++)
                keypoints[joint] = WorldKeypoint(0, joint);

            RestoreBind();
            AlignRootToBodyAnchors(keypoints);
            DriveHips(keypoints);
            for (int index = 0; index < legs.Length; index++)
            {
                ResolvedLeg leg = legs[index];
                Vector3 normal = LegNormal(keypoints, leg);
                if (prevLegNormal != null)
                    prevLegNormal[index] = normal;   // seed FK continuity at calibration sign
                Vector3 upperDirection =
                    keypoints[(int)leg.poleJoint] - keypoints[(int)leg.kneeJoint];
                Vector3 lowerDirection =
                    keypoints[(int)leg.targetJoint] - keypoints[(int)leg.poleJoint];
                legCalUpper[index] =
                    Quaternion.Inverse(Quaternion.LookRotation(upperDirection, normal)) *
                    leg.upper.rotation;
                legCalLower[index] =
                    Quaternion.Inverse(Quaternion.LookRotation(lowerDirection, normal)) *
                    leg.lower.rotation;
            }
        }

        static Vector3 LegNormal(Vector3[] keypoints, ResolvedLeg leg)
        {
            Vector3 upper =
                keypoints[(int)leg.poleJoint] - keypoints[(int)leg.kneeJoint];
            Vector3 lower =
                keypoints[(int)leg.targetJoint] - keypoints[(int)leg.poleJoint];
            Vector3 normal = Vector3.Cross(upper, lower);
            return normal.sqrMagnitude > 1e-8f ? normal.normalized : Vector3.up;
        }

        /// <summary>
        /// Leg-plane normal for FK with sign continuity. When the leg straightens the
        /// two segments become collinear and the raw cross product collapses or flips,
        /// which snaps the LookRotation roll (a visible twist pop). Carry the previous
        /// frame's normal through the degenerate span and keep the sign consistent with
        /// the calibration frame.
        /// </summary>
        Vector3 StableLegNormal(int index, Vector3[] keypoints, ResolvedLeg leg)
        {
            Vector3 upper =
                keypoints[(int)leg.poleJoint] - keypoints[(int)leg.kneeJoint];
            Vector3 lower =
                keypoints[(int)leg.targetJoint] - keypoints[(int)leg.poleJoint];
            Vector3 normal = Vector3.Cross(upper, lower);
            Vector3 prev = prevLegNormal != null ? prevLegNormal[index] : Vector3.zero;

            if (normal.sqrMagnitude <= 1e-8f)
                return prev != Vector3.zero ? prev : Vector3.up;

            normal.Normalize();
            if (prev != Vector3.zero && Vector3.Dot(normal, prev) < 0f)
                normal = -normal;
            if (prevLegNormal != null)
                prevLegNormal[index] = normal;
            return normal;
        }

        protected virtual void ApplyScale()
        {
            dogRoot.localScale = rootBindLocalScale;
            if (!autoScaleDogToCapture)
            {
                dogRoot.localScale *= scaleMultiplier;
                return;
            }

            float dogTrunk = (throatBone.position - tailBone.position).magnitude;
            float captureTrunk =
                (WorldKeypoint(0, (int)DogKeypoint.Throat) -
                 WorldKeypoint(0, (int)DogKeypoint.TailBase)).magnitude;
            if (dogTrunk < 1e-5f || captureTrunk < 1e-5f)
                throw new InvalidOperationException(
                    "DogPoseDriver: degenerate trunk length; cannot auto-scale");
            dogRoot.localScale =
                rootBindLocalScale * (captureTrunk / dogTrunk) * scaleMultiplier;
        }

        protected void Update()
        {
            if (clipIndex != loadedIndex)
                LoadClip(clipIndex);

            float fps = fpsOverride > 0f ? fpsOverride : clip.fps;
            time += Time.deltaTime;
            float framePos = time * fps;
            int frame = Mathf.FloorToInt(framePos);
            float frac = framePos - frame;
            int next;
            if (loop)
            {
                frame = ((frame % clip.frameCount) + clip.frameCount) % clip.frameCount;
                next = (frame + 1) % clip.frameCount;
            }
            else
            {
                frame = Mathf.Min(frame, clip.frameCount - 1);
                next = Mathf.Min(frame + 1, clip.frameCount - 1);
                if (frame == next)
                    frac = 0f;
            }
            Pose(frame, next, frac);
        }

        void Pose(int frame, int next, float frac)
        {
            Vector3[] keypoints = ResolveKeypoints(frame, next, frac);
            gizmoKeypoints = keypoints;
            gizmoFrame = frame;

            RestoreBind();
            if (alignRootToTrunk)
                AlignRootToBodyAnchors(keypoints);
            PlaceRootBetweenBodyAnchors(keypoints);
            ApplyAims(keypoints);
            DriveHips(keypoints);
            SolveLegs(keypoints);
        }

        Vector3[] ResolveKeypoints(int frame, int next, float frac)
        {
            float dt = Time.deltaTime;
            var keypoints = new Vector3[DogJoints.Count];
            for (int joint = 0; joint < DogJoints.Count; joint++)
            {
                bool weak = holdLastValid && frame > 0 &&
                            clip.Confidence(frame, joint) < confidenceThreshold;
                Vector3 pos;
                if (weak)
                {
                    pos = lastValidWorld[joint];   // already smoothed / held
                }
                else
                {
                    // Interpolate between whole frames so playback is not stepped,
                    // then adaptively low-pass to kill capture jitter.
                    pos = Vector3.Lerp(
                        WorldKeypoint(frame, joint), WorldKeypoint(next, joint), frac);
                    if (smoothKeypoints && keypointFilters != null)
                        pos = keypointFilters[joint].Filter(pos, dt);
                }
                keypoints[joint] = pos;
                lastValidWorld[joint] = pos;
            }
            return keypoints;
        }

        Vector3 WorldKeypoint(int frame, int joint)
        {
            Vector3 raw = clip.Position(frame, joint);
            Vector3 value = swapYZ ? new Vector3(raw.x, raw.z, raw.y) : raw;
            value.Scale(axisSign);
            return captureOrigin != null ? captureOrigin.TransformPoint(value) : value;
        }

        void AlignRootToBodyAnchors(Vector3[] keypoints)
        {
            Vector3 dogTrunk = throatBone.position - tailBone.position;
            Vector3 captureTrunk =
                keypoints[(int)DogKeypoint.Throat] -
                keypoints[(int)DogKeypoint.TailBase];
            if (dogTrunk.sqrMagnitude > 1e-8f && captureTrunk.sqrMagnitude > 1e-8f)
            {
                Quaternion delta = Quaternion.FromToRotation(dogTrunk, captureTrunk);
                delta = RootAxisMask.Rotation(delta, alignRotationAxes);
                dogRoot.rotation = Quaternion.Slerp(
                    Quaternion.identity, delta, Mathf.Clamp01(bodyRotationWeight)) *
                    dogRoot.rotation;
            }
        }

        void PlaceRootBetweenBodyAnchors(Vector3[] keypoints)
        {
            Vector3 dogCenter = (throatBone.position + tailBone.position) * 0.5f;
            Vector3 captureCenter =
                (keypoints[(int)DogKeypoint.Throat] +
                 keypoints[(int)DogKeypoint.TailBase]) * 0.5f;
            Vector3 delta = (captureCenter - dogCenter) *
                            Mathf.Clamp01(bodyPositionWeight);
            dogRoot.position += RootAxisMask.Vector(delta, alignTranslationAxes);
        }

        /// <summary>The rig root this driver places for root motion.</summary>
        public Transform BodyRoot => dogRoot != null ? dogRoot : transform;

        /// <summary>Current body frame from the posed trunk: centre = midpoint of
        /// Throat and TailBase, forward = TailBase -> Throat. Used by
        /// <see cref="RootMotionAnchor"/> to re-base the root motion.</summary>
        public bool TryGetBodyFrame(out Vector3 origin, out Vector3 forward)
        {
            if (throatBone == null || tailBone == null)
            {
                origin = default;
                forward = default;
                return false;
            }
            origin = (throatBone.position + tailBone.position) * 0.5f;
            forward = throatBone.position - tailBone.position;
            return forward.sqrMagnitude > 1e-8f;
        }

        void ApplyAims(Vector3[] keypoints)
        {
            if (aimHead)
                AimBone(
                    throatBone,
                    headAimBone,
                    keypoints[(int)headAimKeypoint] -
                    keypoints[(int)DogKeypoint.Throat],
                    headAimWeight);
        }

        /// <summary>
        /// Articulate each leg's femur (body->Knee segment). The captured skeleton
        /// has three moving segments per leg (anchor->Knee, Knee->Elbow, Elbow->Paw),
        /// but the two-bone leg IK only covers the lower two. Without this the Knee
        /// bone rides rigidly with the body, discarding the largest, most-mobile
        /// segment and pushing the paw target out of reach (worst on the hind legs).
        /// Aiming the femur here re-anchors the leg IK at the correct Knee position.
        /// </summary>
        void DriveHips(Vector3[] keypoints)
        {
            if (!driveHip)
                return;
            for (int index = 0; index < legs.Length; index++)
            {
                ResolvedLeg leg = legs[index];
                if (leg.femur == null)
                    continue;
                float weight = hipAimWeight * LegWeight(index);
                if (weight <= 0f)
                    continue;
                AimBone(
                    leg.femur,
                    leg.upper,
                    keypoints[(int)leg.kneeJoint] - keypoints[(int)leg.hipAnchor],
                    weight);
            }
        }

        static void AimBone(Transform bone, Transform child, Vector3 worldDirection,
                            float weight)
        {
            Vector3 currentDirection = child.position - bone.position;
            if (currentDirection.sqrMagnitude > 1e-8f &&
                worldDirection.sqrMagnitude > 1e-8f)
            {
                Quaternion delta = Quaternion.FromToRotation(
                    currentDirection, worldDirection);
                bone.rotation = Quaternion.Slerp(
                    Quaternion.identity, delta, Mathf.Clamp01(weight)) * bone.rotation;
            }
        }

        void SolveLegs(Vector3[] keypoints)
        {
            for (int index = 0; index < legs.Length; index++)
            {
                ResolvedLeg leg = legs[index];
                float legWeight = LegWeight(index);
                if (legWeight <= 0f)
                    continue;

                Quaternion upperBefore = leg.upper.rotation;
                Quaternion lowerBefore = leg.lower.rotation;
                if (legMode == LegMode.FK)
                {
                    Vector3 normal = StableLegNormal(index, keypoints, leg);
                    Vector3 upperDirection =
                        keypoints[(int)leg.poleJoint] -
                        keypoints[(int)leg.kneeJoint];
                    Vector3 lowerDirection =
                        keypoints[(int)leg.targetJoint] -
                        keypoints[(int)leg.poleJoint];
                    if (upperDirection.sqrMagnitude > 1e-8f)
                        leg.upper.rotation =
                            Quaternion.LookRotation(upperDirection, normal) *
                            legCalUpper[index];
                    if (lowerDirection.sqrMagnitude > 1e-8f)
                        leg.lower.rotation =
                            Quaternion.LookRotation(lowerDirection, normal) *
                            legCalLower[index];

                    // Plant the FK foot during stance with a light IK correction, so a
                    // planted paw stops sliding without disturbing the swing phase.
                    if (lockPlantedFeet && footPlanters != null)
                    {
                        Vector3 detect = keypoints[(int)leg.targetJoint];
                        Vector3 plant = footPlanters[index].Filter(
                            leg.end.position, detect, Time.deltaTime,
                            footLockSpeed, footUnlockSpeed, footLockAttack);
                        if ((plant - leg.end.position).sqrMagnitude > 1e-10f)
                            TwoBoneIK.Solve(leg.upper, leg.lower, leg.end,
                                            plant, leg.lower.position, ikSoftZone);
                    }
                }
                else
                {
                    GetLegIKTarget(index, leg, keypoints,
                                   out Vector3 target, out Vector3 pole);
                    pole = Vector3.Lerp(
                        leg.lower.position, pole, PoleWeight(index));
                    if (lockPlantedFeet && footPlanters != null)
                        target = footPlanters[index].Filter(
                            target, keypoints[(int)leg.targetJoint], Time.deltaTime,
                            footLockSpeed, footUnlockSpeed, footLockAttack);
                    TwoBoneIK.Solve(
                        leg.upper,
                        leg.lower,
                        leg.end,
                        target,
                        pole,
                        ikSoftZone);
                    if (limitIkLegRoll)
                    {
                        LimitRoll(leg.upper, bindLocalUpper[index],
                                  rollAxisUpper[index], maxIkLegRollDegrees);
                        LimitRoll(leg.lower, bindLocalLower[index],
                                  rollAxisLower[index], maxIkLegRollDegrees);
                    }
                }

                if (legWeight < 1f)
                {
                    Quaternion upperSolved = leg.upper.rotation;
                    Quaternion lowerSolved = leg.lower.rotation;
                    leg.upper.rotation = Quaternion.Slerp(
                        upperBefore, upperSolved, legWeight);
                    leg.lower.rotation = Quaternion.Slerp(
                        lowerBefore, lowerSolved, legWeight);
                }
            }
        }

        static void LimitRoll(Transform bone, Quaternion bindLocal,
                              Vector3 axisLocal, float maxDegrees)
        {
            Quaternion delta = Quaternion.Inverse(bindLocal) * bone.localRotation;
            Vector3 projected = Vector3.Project(
                new Vector3(delta.x, delta.y, delta.z), axisLocal);
            float normSquared = projected.sqrMagnitude + delta.w * delta.w;
            Quaternion twist;
            if (normSquared < 1e-8f)
            {
                twist = Quaternion.identity;
            }
            else
            {
                float inverseNorm = 1f / Mathf.Sqrt(normSquared);
                twist = new Quaternion(
                    projected.x * inverseNorm,
                    projected.y * inverseNorm,
                    projected.z * inverseNorm,
                    delta.w * inverseNorm);
            }

            Quaternion swing = delta * Quaternion.Inverse(twist);
            twist = Quaternion.RotateTowards(
                Quaternion.identity, twist, Mathf.Clamp(maxDegrees, 0f, 90f));
            bone.localRotation = bindLocal * swing * twist;
        }

        float LegWeight(int index)
        {
            switch (index)
            {
                case 0: return Mathf.Clamp01(leftFrontLegWeight);
                case 1: return Mathf.Clamp01(rightFrontLegWeight);
                case 2: return Mathf.Clamp01(leftBackLegWeight);
                case 3: return Mathf.Clamp01(rightBackLegWeight);
                default: throw new ArgumentOutOfRangeException(nameof(index), index, null);
            }
        }

        float PoleWeight(int index)
        {
            switch (index)
            {
                case 0: return Mathf.Clamp01(leftFrontPoleWeight);
                case 1: return Mathf.Clamp01(rightFrontPoleWeight);
                case 2: return Mathf.Clamp01(leftBackPoleWeight);
                case 3: return Mathf.Clamp01(rightBackPoleWeight);
                default: throw new ArgumentOutOfRangeException(nameof(index), index, null);
            }
        }

        protected virtual void GetLegIKTarget(int index, ResolvedLeg leg,
                                              Vector3[] keypoints,
                                              out Vector3 target, out Vector3 pole)
        {
            target = keypoints[(int)leg.targetJoint];
            pole = keypoints[(int)leg.poleJoint];
        }

        protected void OnDrawGizmos()
        {
            if (!drawKeypointGizmos || clip == null || gizmoKeypoints == null)
                return;

            Gizmos.color = Color.cyan;
            for (int joint = 0; joint < DogJoints.Count; joint++)
            {
                int parent = clip.parents[joint];
                if (parent >= 0)
                    Gizmos.DrawLine(gizmoKeypoints[joint], gizmoKeypoints[parent]);
            }
            for (int joint = 0; joint < DogJoints.Count; joint++)
            {
                Color color = joint >= (int)DogKeypoint.L_F_Paw
                    ? Color.red
                    : Color.yellow;
                color.a = Mathf.Max(0.25f, clip.Confidence(gizmoFrame, joint));
                Gizmos.color = color;
                Gizmos.DrawSphere(gizmoKeypoints[joint], gizmoJointRadius);
#if UNITY_EDITOR
                if (drawJointLabels)
                    UnityEditor.Handles.Label(
                        gizmoKeypoints[joint], ((DogKeypoint)joint).ToString());
#endif
            }
        }
    }
}
