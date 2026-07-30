using System;
using UnityEngine;

namespace PetDemo
{
    /// <summary>
    /// Cyclic-coordinate-descent IK for a bone chain of any length. Rotates each
    /// bone in <c>chain</c> (ordered base -> tip) so <c>effector</c> reaches
    /// <c>target</c>. An optional <c>pole</c> then twists the whole chain about
    /// the base->target axis so its tip joint bends toward the pole, which fixes
    /// the bend plane (a knee/elbow direction). Rotation only: the existing bone
    /// lengths are never changed, so the rig keeps its proportions.
    /// </summary>
    public static class ChainIK
    {
        const float Eps = 1e-6f;

        /// <param name="chain">Rotatable bones, base first. The effector must be a
        /// descendant of the last bone.</param>
        /// <param name="effector">End joint driven onto <paramref name="target"/>.</param>
        /// <param name="target">Desired world position of the effector.</param>
        /// <param name="iterations">CCD sweeps (higher = tighter convergence).</param>
        /// <param name="pole">World hint the tip joint bends toward, or null.</param>
        public static void Solve(Transform[] chain, Transform effector,
                                 Vector3 target, int iterations, Vector3? pole = null)
        {
            if (chain == null || chain.Length == 0)
                throw new ArgumentException("ChainIK: empty chain");

            for (int it = 0; it < iterations; it++)
            {
                for (int i = chain.Length - 1; i >= 0; i--)
                {
                    Transform b = chain[i];
                    Vector3 toEff = effector.position - b.position;
                    Vector3 toTgt = target - b.position;
                    if (toEff.sqrMagnitude < Eps || toTgt.sqrMagnitude < Eps)
                        continue;
                    b.rotation = Quaternion.FromToRotation(toEff, toTgt) * b.rotation;
                }
                if ((effector.position - target).sqrMagnitude < Eps)
                    break;
            }

            if (pole.HasValue)
                OrientToPole(chain, target, pole.Value);
        }

        /// <summary>
        /// Twist the chain about the base->target axis so the tip joint (the last
        /// bone's head, i.e. the joint just above the effector) points toward
        /// <paramref name="pole"/>. The axis passes through the target, so the
        /// effector barely moves.
        /// </summary>
        static void OrientToPole(Transform[] chain, Vector3 target, Vector3 pole)
        {
            Transform baseBone = chain[0];
            Vector3 axis = target - baseBone.position;
            if (axis.sqrMagnitude < Eps)
                return;
            axis.Normalize();

            Vector3 tip = chain[chain.Length - 1].position - baseBone.position;
            Vector3 wanted = pole - baseBone.position;
            tip -= Vector3.Dot(tip, axis) * axis;       // project off the axis
            wanted -= Vector3.Dot(wanted, axis) * axis;
            if (tip.sqrMagnitude < Eps || wanted.sqrMagnitude < Eps)
                return;

            float twist = Vector3.SignedAngle(tip, wanted, axis);
            baseBone.rotation = Quaternion.AngleAxis(twist, axis) * baseBone.rotation;
        }
    }
}
