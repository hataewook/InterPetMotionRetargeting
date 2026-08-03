using UnityEngine;

namespace PetDemo
{
    /// <summary>
    /// Analytic two-bone IK. Rotates <c>upper</c> and <c>lower</c> (a contiguous
    /// parent->child chain) so the tip reaches <c>target</c>, bending toward
    /// <c>pole</c>. Rotation only: the existing bone lengths are never changed,
    /// which is what keeps the dog's proportions intact.
    /// </summary>
    public static class TwoBoneIK
    {
        const float Eps = 1e-5f;

        // Below this sin(theta) between (target-a) and (pole-a) the pole/target plane
        // is too collinear to define a bend side (a near-fully-extended limb), so the
        // pole-axis overload holds its last good plane normal rather than chasing the
        // (now ill-conditioned, noisy) pole direction. ~sin(14.5 deg).
        const float PoleAxisMinSin = 0.25f;

        /// <param name="upper">Chain root (knee bone).</param>
        /// <param name="lower">Middle joint (elbow bone), child of upper.</param>
        /// <param name="tip">End effector (paw _end bone), child of lower.</param>
        /// <param name="target">Desired world position of the tip.</param>
        /// <param name="pole">World hint the bend (elbow) points toward.</param>
        /// <param name="softZone">Fraction of the total leg length over which the
        /// reach eases into full extension instead of snapping straight (0 = the old
        /// hard clamp). A small value, e.g. 0.03, removes the "knee pop" as the foot
        /// goal approaches the limb's maximum length while leaving normal reaches
        /// untouched.</param>
        public static void Solve(Transform upper, Transform lower, Transform tip,
                                 Vector3 target, Vector3 pole, float softZone = 0f)
        {
            Vector3 unused = Vector3.zero;
            SolveInternal(upper, lower, tip, target, pole, softZone, false, ref unused);
        }

        /// <summary>
        /// As <see cref="Solve(Transform,Transform,Transform,Vector3,Vector3,float)"/>,
        /// but derives the bend axis from the pole/target plane rather than the current
        /// (possibly flipped) pose, and holds it through the near-straight span where
        /// that plane is ill-defined. This stops a near-fully-extended limb (e.g. a
        /// columnar front leg) from flipping its bend side frame to frame. Pass a
        /// per-limb <see cref="Vector3"/> that persists across frames, seeded to
        /// <see cref="Vector3.zero"/>; it is read and updated in place.
        /// </summary>
        public static void Solve(Transform upper, Transform lower, Transform tip,
                                 Vector3 target, Vector3 pole, float softZone,
                                 ref Vector3 bendAxisState)
        {
            SolveInternal(upper, lower, tip, target, pole, softZone, true, ref bendAxisState);
        }

        static void SolveInternal(Transform upper, Transform lower, Transform tip,
                                  Vector3 target, Vector3 pole, float softZone,
                                  bool poleBendAxis, ref Vector3 bendAxisState)
        {
            Vector3 a = upper.position;
            Vector3 b = lower.position;
            Vector3 c = tip.position;

            float lenAB = (b - a).magnitude;
            float lenBC = (c - b).magnitude;
            if (lenAB < Eps || lenBC < Eps)
                return;

            // Clamp reach so the target is always solvable (no stretching). Near full
            // extension, optionally ease in so the leg approaches (but never quite
            // reaches) its length asymptotically rather than snapping locked.
            float maxLen = lenAB + lenBC;
            float reach = (target - a).magnitude;
            float lenAT;
            if (softZone > 0f)
            {
                float soft = maxLen * (1f - Mathf.Clamp01(softZone));
                lenAT = reach <= soft
                    ? reach
                    : soft + (maxLen - soft) *
                             (1f - Mathf.Exp(-(reach - soft) / Mathf.Max(maxLen - soft, Eps)));
            }
            else
            {
                lenAT = reach;
            }
            lenAT = Mathf.Clamp(lenAT, Eps, maxLen - Eps);

            // Desired interior angles from the law of cosines.
            float wantBAT = Mathf.Acos(Mathf.Clamp(
                (lenAB * lenAB + lenAT * lenAT - lenBC * lenBC) / (2f * lenAB * lenAT), -1f, 1f));
            float wantABC = Mathf.Acos(Mathf.Clamp(
                (lenAB * lenAB + lenBC * lenBC - lenAT * lenAT) / (2f * lenAB * lenBC), -1f, 1f));

            // Current interior angles.
            float curBAC = Mathf.Acos(Mathf.Clamp(
                Vector3.Dot((c - a).normalized, (b - a).normalized), -1f, 1f));
            float curABC = Mathf.Acos(Mathf.Clamp(
                Vector3.Dot((a - b).normalized, (c - b).normalized), -1f, 1f));

            // The bend plane's normal defines which way the elbow/knee folds. We take
            // it from the plane through the aim (target) and the desired bend (pole),
            // so its sign is anchored to where we WANT the elbow -- a flipped current
            // pose can never flip the bend side. When target and pole are nearly
            // collinear (a near-straight limb) that plane is undefined, so we hold the
            // last good normal instead of letting the ill-conditioned pole flip it.
            if (poleBendAxis)
            {
                Vector3 n;                                          // bend plane normal
                Vector3 poleAxis = Vector3.Cross(target - a, pole - a);
                float denom = (target - a).magnitude * (pole - a).magnitude;
                float sinTheta = denom > Eps ? poleAxis.magnitude / denom : 0f;
                bool haveNormal = true;
                if (sinTheta > PoleAxisMinSin)
                {
                    n = poleAxis / poleAxis.magnitude;              // strong pole signal
                    // A quadruped leg never reverses which way it folds, so a sign flip
                    // here is pole noise, not real -- snap back to the last good side.
                    if (bendAxisState.sqrMagnitude > Eps && Vector3.Dot(n, bendAxisState) < 0f)
                        n = -n;
                    bendAxisState = n;
                }
                else if (bendAxisState.sqrMagnitude > Eps)
                {
                    n = bendAxisState;                              // near-straight: hold
                }
                else
                {
                    n = Vector3.zero;                               // no plane established yet
                    haveNormal = false;
                }

                if (haveNormal)
                {
                    // Absolute, roll-free placement: put the elbow in the held plane at
                    // the solved interior angle, on the pole side, using shortest-arc
                    // swings only. Because no twist about the bone's length axis is
                    // introduced, a downstream roll-lock (LockRoll) has nothing to fight,
                    // which is what previously snapped the elbow back to the wrong side.
                    Vector3 aim = target - a;
                    if (aim.sqrMagnitude < Eps)
                        return;
                    Vector3 elbowDir = Quaternion.AngleAxis(wantBAT * Mathf.Rad2Deg, n) *
                                       aim.normalized;              // rotate aim toward pole
                    upper.rotation = Quaternion.FromToRotation(b - a, elbowDir) * upper.rotation;

                    Vector3 bNew = lower.position;
                    Vector3 lowerDir = tip.position - bNew;
                    if (lowerDir.sqrMagnitude > Eps)
                        lower.rotation =
                            Quaternion.FromToRotation(lowerDir, target - bNew) * lower.rotation;
                    return;
                }
                // else: no plane yet (first frames near-straight) -> original solve below.
            }

            // Default incremental solve (the un-refed overload, and the pole overload's
            // first-frame fallback): bend about the current triangle normal, aim, then
            // twist toward the pole.
            Vector3 axis = Vector3.Cross(c - a, b - a);
            if (axis.sqrMagnitude < Eps)
                axis = Vector3.Cross(target - a, pole - a);
            axis.Normalize();

            // Apply the two bends in local space so children ride along.
            upper.rotation = Quaternion.AngleAxis(
                (wantBAT - curBAC) * Mathf.Rad2Deg, axis) * upper.rotation;
            lower.rotation = Quaternion.AngleAxis(
                (wantABC - curABC) * Mathf.Rad2Deg, axis) * lower.rotation;

            // Aim the (now correctly-bent) limb straight at the target.
            Vector3 tipDir = tip.position - a;
            if (tipDir.sqrMagnitude > Eps)
                upper.rotation = Quaternion.FromToRotation(tipDir, target - a) * upper.rotation;

            // Twist about the A->target axis so the elbow points toward the pole.
            OrientToPole(upper, lower, a, target, pole);
        }

        static void OrientToPole(Transform upper, Transform lower,
                                 Vector3 a, Vector3 target, Vector3 pole)
        {
            Vector3 axis = (target - a).normalized;
            if (axis.sqrMagnitude < Eps)
                return;

            Vector3 elbow = (lower.position - a);
            Vector3 wanted = (pole - a);
            elbow -= Vector3.Dot(elbow, axis) * axis;   // project off the aim axis
            wanted -= Vector3.Dot(wanted, axis) * axis;
            if (elbow.sqrMagnitude < Eps || wanted.sqrMagnitude < Eps)
                return;

            float twist = Vector3.SignedAngle(elbow, wanted, axis);
            upper.rotation = Quaternion.AngleAxis(twist, axis) * upper.rotation;
        }
    }
}
