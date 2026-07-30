using System;
using UnityEngine;

namespace PetDemo
{
    /// <summary>
    /// A leg solved with analytic two-bone IK: <see cref="upperBone"/> ->
    /// <see cref="lowerBone"/> -> <see cref="tipBone"/> is a contiguous
    /// parent->child chain whose tip is driven onto SMAL joint
    /// <see cref="targetJoint"/>, bending toward SMAL joint <see cref="poleJoint"/>
    /// (the knee/elbow) so the bend never folds or twists the wrong way. The foot
    /// goal is placed relative to SMAL joint <see cref="rootJoint"/> (the shoulder/
    /// hip) and normalized by the leg's own length, so it can never reach past the
    /// target leg — the limb keeps its size regardless of the source's build.
    ///
    /// An optional <see cref="toeBone"/> (a descendant of the tip, e.g. the paw/toe)
    /// is then oriented per segment FK to follow the SMAL segment
    /// <see cref="targetJoint"/> -> <see cref="toeJoint"/>: the IK still plants the
    /// paw, and the toe adds articulation on top without disturbing the plant.
    /// Leave <see cref="toeBone"/> unassigned to skip it; <see cref="toeJoint"/> = 0
    /// defaults to <see cref="targetJoint"/> + 1 (the paw's SMAL child).
    /// </summary>
    [Serializable]
    public struct RetargetLeg
    {
        public string name;
        public Transform upperBone;
        public Transform lowerBone;
        public Transform tipBone;
        [Tooltip("Optional paw/toe bone (a descendant of the tip) driven by FK after the " +
                 "IK plants the paw. Leave empty to skip.")]
        public Transform toeBone;
        public int rootJoint;
        public int targetJoint;
        public int poleJoint;
        [Tooltip("SMAL joint beyond the paw that the toe points at. 0 = targetJoint + 1.")]
        public int toeJoint;

        public RetargetLeg(string name, Transform upperBone, Transform lowerBone, Transform tipBone,
                           int rootJoint, int targetJoint, int poleJoint,
                           Transform toeBone = null, int toeJoint = 0)
        {
            this.name = name; this.upperBone = upperBone;
            this.lowerBone = lowerBone; this.tipBone = tipBone;
            this.rootJoint = rootJoint; this.targetJoint = targetJoint; this.poleJoint = poleJoint;
            this.toeBone = toeBone; this.toeJoint = toeJoint;
        }
    }

    /// <summary>
    /// A non-leg chain (trunk/neck/head) solved with CCD: the bones from
    /// <see cref="baseBone"/> down to <see cref="effectorBone"/> are rotated so the
    /// effector reaches a goal placed relative to SMAL joint <see cref="rootJoint"/>
    /// (the chain base) and normalized by the chain's own length, driven from SMAL
    /// joint <see cref="endJoint"/>.
    /// </summary>
    [Serializable]
    public struct RetargetChain
    {
        public string name;
        public Transform baseBone;
        public Transform effectorBone;
        public int rootJoint;
        public int endJoint;

        public RetargetChain(string name, Transform baseBone, Transform effectorBone,
                             int rootJoint, int endJoint)
        {
            this.name = name; this.baseBone = baseBone; this.effectorBone = effectorBone;
            this.rootJoint = rootJoint; this.endJoint = endJoint;
        }
    }

    /// <summary>
    /// A segment-by-segment FK chain — spine, neck or tail — retargeted per segment:
    /// the SMAL joint chain <see cref="joints"/> (base -> tip) defines the source
    /// segment directions, and each target bone in <see cref="bones"/> (base ->
    /// tip, a contiguous parent->child chain) is rotated so it follows the
    /// direction of its corresponding SMAL segment, carried through the body frame.
    /// The solve is rotation only, so each target bone keeps its own length and any
    /// length difference between the rigs is absorbed automatically — no scaling
    /// needed. Roll is inherited from the bind pose (only the minimal swing that
    /// aligns the direction is applied), so a bending/wagging chain never twists.
    /// When the bone and SMAL-segment counts differ, the target bones are
    /// distributed proportionally across the SMAL segments (exact 1:1 when
    /// <c>joints.Length == bones.Length + 1</c>). Unlike the leg two-bone IK and the
    /// trunk CCD, this reproduces the whole curve of the chain, not just its endpoint.
    /// </summary>
    [Serializable]
    public struct RetargetFkChain
    {
        public string name;
        [Tooltip("Target bones, base -> tip. Each is a contiguous parent->child " +
                 "chain and is oriented to follow its SMAL segment.")]
        public Transform[] bones;
        [Tooltip("SMAL joint indices, base -> tip. Segment k is joints[k]->joints[k+1]; " +
                 "there must be at least two. Leave empty and the name (spine/neck/tail) " +
                 "picks the default SMAL chain.")]
        public int[] joints;

        public RetargetFkChain(string name, Transform[] bones, int[] joints)
        {
            this.name = name; this.bones = bones; this.joints = joints;
        }
    }

    /// <summary>A signed target-local axis, used to remap a SMAL rotation axis onto a
    /// differently-oriented target bone.</summary>
    public enum Axis { XPlus, XMinus, YPlus, YMinus, ZPlus, ZMinus }

    /// <summary>
    /// Copies a single SMAL joint's local articulation (its rotation relative to its
    /// neutral rest pose) onto <see cref="targetBone"/> — used for the jaw and ears
    /// (SMAL joints 32/33/34), which have no length to retarget, only rotation. The
    /// reference is the true rest pose (from the SmalMotionPlayer), so only the change
    /// from neutral is transferred, not a raw rotation value.
    ///
    /// Because the two rigs' bones need not share an orientation, the SMAL rotation
    /// is copied as an axis-angle and its axis is remapped: <see cref="mapX"/> /
    /// <see cref="mapY"/> / <see cref="mapZ"/> say which signed target-local axis the
    /// SMAL local X / Y / Z rotation drives. Identity (X+, Y+, Z+) copies verbatim;
    /// flip or swap them until the jaw opens and the ears turn the right way.
    /// <see cref="weight"/> scales the copied angle: 1 = 1:1, &lt;1 suppresses the
    /// motion, 0 freezes the bone at its bind, &gt;1 exaggerates it.
    /// </summary>
    [Serializable]
    public struct RetargetCopyRotation
    {
        public string name;
        [Tooltip("Target bone that receives the copied rotation (e.g. jaw / ear bone).")]
        public Transform targetBone;
        [Tooltip("SMAL joint whose local rotation is copied (jaw 32, L ear 33, R ear 34).")]
        public int joint;
        [Tooltip("Target-local axis driven by the SMAL local X rotation.")]
        public Axis mapX;
        [Tooltip("Target-local axis driven by the SMAL local Y rotation.")]
        public Axis mapY;
        [Tooltip("Target-local axis driven by the SMAL local Z rotation.")]
        public Axis mapZ;
        [Tooltip("Scale on the copied angle. 1 = 1:1, <1 suppress, 0 = freeze at bind, " +
                 ">1 exaggerate. (A loaded 0 is treated as 1 so new entries copy 1:1.)")]
        [Range(0f, 2f)] public float weight;

        public RetargetCopyRotation(string name, Transform targetBone, int joint,
                                    Axis mapX = Axis.XPlus, Axis mapY = Axis.YPlus, Axis mapZ = Axis.ZPlus,
                                    float weight = 1f)
        {
            this.name = name; this.targetBone = targetBone; this.joint = joint;
            this.mapX = mapX; this.mapY = mapY; this.mapZ = mapZ; this.weight = weight;
        }

        /// <summary>Unit vector for a signed axis.</summary>
        public static Vector3 AxisVec(Axis a)
        {
            switch (a)
            {
                case Axis.XPlus:  return Vector3.right;
                case Axis.XMinus: return Vector3.left;
                case Axis.YPlus:  return Vector3.up;
                case Axis.YMinus: return Vector3.down;
                case Axis.ZPlus:  return Vector3.forward;
                case Axis.ZMinus: return Vector3.back;
                default:          return Vector3.right;
            }
        }
    }

    /// <summary>
    /// Default SMAL-35 source joint indices used to seed the Inspector. Target
    /// transforms are assigned per model. SMAL joint indices follow the InterPet4D
    /// neutral rig. The four torso-frame joints (shoulders 7/11, hips 17/21) are
    /// consumed directly by <see cref="SmalRetargeter"/> to build the body frame.
    /// </summary>
    public static class SmalRetargetMap
    {
        // SMAL joints defining the torso frame: forward = hips -> shoulders,
        // lateral = left shoulder -> right shoulder.
        public const int LeftShoulder = 7, RightShoulder = 11;
        public const int LeftHip = 17, RightHip = 21;

        // SMAL face joints copied by rotation only (no length to retarget).
        public const int Jaw = 32, LeftEar = 33, RightEar = 34;

        // SMAL FK chains, base -> tip. Spine climbs the back from the pelvis to the
        // chest; neck runs chest -> head base; tail runs off the pelvis.
        public static readonly int[] SpineJoints = { 1, 2, 3, 4, 5, 6 };
        public static readonly int[] NeckJoints = { 15, 16 };
        public static readonly int[] TailJoints = { 25, 26, 27, 28, 29, 30, 31 };

        /// <summary>Default SMAL joint chain for an FK chain, picked from its name
        /// (spine / neck / tail). Returns null when the name matches none, so the
        /// caller can require the joints be set explicitly.</summary>
        public static int[] DefaultJoints(string name)
        {
            if (string.IsNullOrEmpty(name)) return null;
            string n = name.ToLowerInvariant();
            if (n.Contains("spine") || n.Contains("back") || n.Contains("척추"))
                return (int[])SpineJoints.Clone();
            if (n.Contains("neck") || n.Contains("목"))
                return (int[])NeckJoints.Clone();
            if (n.Contains("tail") || n.Contains("꼬리"))
                return (int[])TailJoints.Clone();
            return null;
        }

        public static RetargetLeg[] DefaultLegs() => new[]
        {
            //                name       upper lower tip root tgt pole
            new RetargetLeg("L_front", null, null, null, 7,  9,  8),
            new RetargetLeg("R_front", null, null, null, 11, 13, 12),
            new RetargetLeg("L_back",  null, null, null, 17, 19, 18),
            new RetargetLeg("R_back",  null, null, null, 21, 23, 22),
        };

        public static RetargetChain[] DefaultChains() => new[]
        {
            new RetargetChain("Trunk", null, null, 1, 32),
        };

        public static RetargetFkChain[] DefaultFkChains() => new[]
        {
            new RetargetFkChain("Spine", null, (int[])SpineJoints.Clone()),
            new RetargetFkChain("Tail",  null, (int[])TailJoints.Clone()),
        };

        public static RetargetCopyRotation[] DefaultCopyRotations() => new[]
        {
            new RetargetCopyRotation("Jaw",   null, Jaw),
            new RetargetCopyRotation("L_Ear", null, LeftEar),
            new RetargetCopyRotation("R_Ear", null, RightEar),
        };
    }
}
