using System;
using System.Collections.Generic;
using UnityEngine;

namespace PetDemo
{
    /// <summary>
    /// Drives the Animation Rigging IK targets on the Shepherd rig from the live SMAL
    /// source rig (posed each frame by <see cref="SmalMotionPlayer"/>). Every mapped
    /// target is moved onto its SMAL joint's world position; the Shepherd's
    /// TwoBoneIKConstraints then solve the legs / nose / tail / trunk to reach them.
    /// Root motion is transferred by making the Shepherd root follow the SMAL body's
    /// translation and rotation, so the whole animal moves through the scene (the IK
    /// only rotates bones and cannot translate the body on its own).
    ///
    /// Runs in Update at a high execution order, so the SMAL bones are already posed
    /// for the frame and the targets are placed <b>before</b> the Animation Rigging
    /// graph evaluates (between Update and LateUpdate) — no one-frame lag.
    ///
    /// Attach to the Animation Rigging Shepherd root and assign the SMAL source rig.
    /// </summary>
    [DefaultExecutionOrder(200)]
    public class SmalIkTargetDriver : MonoBehaviour
    {
        /// <summary>One Animation Rigging IK target and the SMAL joint it follows.</summary>
        [Serializable]
        public struct TargetBinding
        {
            [Tooltip("The IK constraint's Target transform to move.")]
            public Transform target;
            [Tooltip("SMAL joint index this target follows (bone SMAL_joint_NN).")]
            public int joint;
        }

        [Header("Source")]
        [Tooltip("Root of the SMAL rig driven by SmalMotionPlayer (contains the SMAL_joint bones).")]
        [SerializeField] Transform smalSource;
        [Tooltip("Bone name prefix on the source rig; index is two digits (e.g. SMAL_joint_09).")]
        [SerializeField] string boneNamePrefix = "SMAL_joint_";

        [Header("Rig")]
        [Tooltip("Shepherd root that receives root motion. Defaults to this transform.")]
        [SerializeField] Transform targetRoot;

        [Header("Target mapping")]
        [Tooltip("Each IK target and the SMAL joint index it follows.")]
        [SerializeField] TargetBinding[] bindings;
        [Tooltip("Keep each target's start gap from its joint (expressed in the SMAL body " +
                 "frame) instead of snapping the target exactly onto the joint. Useful when " +
                 "the two rigs are not perfectly aligned at bind.")]
        [SerializeField] bool preserveInitialOffset;

        [Header("Root motion")]
        [Tooltip("Move the Shepherd root to follow the SMAL body's translation / rotation.")]
        [SerializeField] bool applyRootMotion = true;
        [SerializeField] bool rootFollowPosition = true;
        [SerializeField] bool rootFollowRotation = true;

        [Header("Debug")]
        [SerializeField] bool drawGizmos = true;
        [SerializeField] float gizmoRadius = 0.02f;

        Transform[] joints;          // resolved SMAL bone per binding
        Vector3[] offsetBody;        // target->joint gap in SMAL body frame at bind
        Vector3 rootPosOffset;       // targetRoot position in smalSource local space at bind
        Quaternion rootRotOffset;    // targetRoot rotation relative to smalSource at bind

        void Awake()
        {
            if (targetRoot == null)
                targetRoot = transform;
            if (smalSource == null)
                throw new MissingReferenceException("SmalIkTargetDriver: no SMAL source assigned");
            if (bindings == null || bindings.Length == 0)
                throw new MissingReferenceException("SmalIkTargetDriver: no target bindings assigned");

            ResolveJoints();
        }

        void Start()
        {
            // SmalMotionPlayer.Awake has posed frame 0; capture the bind relationships
            // so both the root motion and the (optional) per-target offset add only the
            // SMAL body's delta and preserve the initial placement.
            rootPosOffset = smalSource.InverseTransformPoint(targetRoot.position);
            rootRotOffset = Quaternion.Inverse(smalSource.rotation) * targetRoot.rotation;

            offsetBody = new Vector3[bindings.Length];
            Quaternion invBody = Quaternion.Inverse(smalSource.rotation);
            for (int i = 0; i < bindings.Length; i++)
                offsetBody[i] = invBody * (bindings[i].target.position - joints[i].position);
        }

        void ResolveJoints()
        {
            var byName = new Dictionary<string, Transform>();
            foreach (Transform t in smalSource.GetComponentsInChildren<Transform>(true))
                byName[t.name] = t;

            joints = new Transform[bindings.Length];
            for (int i = 0; i < bindings.Length; i++)
            {
                if (bindings[i].target == null)
                    throw new MissingReferenceException(
                        $"SmalIkTargetDriver: binding {i} has no target assigned");
                string wanted = $"{boneNamePrefix}{bindings[i].joint:00}";
                if (!byName.TryGetValue(wanted, out joints[i]))
                    throw new MissingReferenceException(
                        $"SmalIkTargetDriver: source bone '{wanted}' not found under {smalSource.name}");
            }
        }

        void Update()
        {
            // SmalMotionPlayer (default execution order 0) has posed the SMAL bones
            // this frame; this component (order 200) reads them afterwards.
            if (applyRootMotion)
            {
                if (rootFollowPosition)
                    targetRoot.position = smalSource.TransformPoint(rootPosOffset);
                if (rootFollowRotation)
                    targetRoot.rotation = smalSource.rotation * rootRotOffset;
            }

            for (int i = 0; i < joints.Length; i++)
            {
                Vector3 p = joints[i].position;
                if (preserveInitialOffset)
                    p += smalSource.rotation * offsetBody[i];
                bindings[i].target.position = p;
            }
        }

        void OnDrawGizmos()
        {
            if (!drawGizmos || bindings == null)
                return;
            Gizmos.color = Color.magenta;
            foreach (var b in bindings)
                if (b.target != null)
                    Gizmos.DrawSphere(b.target.position, gizmoRadius);
        }
    }
}
