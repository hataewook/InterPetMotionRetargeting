using System;
using UnityEngine;

namespace PetDemo
{
    /// <summary>
    /// A driver (SmalRetargeter / DogPoseDriver) whose posed rig can be settled onto
    /// the ground as a final stage. The driver gathers its own leg data and hands it to
    /// the shared <see cref="FootGrounding"/> logic, then re-applies any pose that
    /// depends on the corrected legs (e.g. toe FK). Same root source as
    /// <see cref="IBodyFrameProvider"/>.
    /// </summary>
    public interface IGroundable
    {
        /// <summary>The rig root the driver moves for root motion.</summary>
        Transform BodyRoot { get; }

        /// <summary>Settle this rig's feet/body onto the ground for the current frame.
        /// Called by <see cref="FootGroundingStage"/> after the driver (and any
        /// <see cref="RootMotionAnchor"/>) have posed the rig.</summary>
        void ApplyFootGrounding(FootGroundingSettings settings, FootGroundingState state);
    }

    /// <summary>
    /// One foot's data for a grounding pass. The driver fills this: <see cref="tip"/>
    /// to read the paw's current world height, <see cref="isFront"/> for the pelvis
    /// front/hind grouping, and <see cref="solveToTip"/> — a closure that re-solves the
    /// leg so the paw reaches a given world position, reusing the driver's own IK
    /// (pole, bend-axis continuity) and roll-lock. Kept as a delegate so this shared
    /// code never touches the driver's private leg internals.
    /// </summary>
    public struct GroundLeg
    {
        public Transform tip;
        public bool isFront;
        public Action<Vector3> solveToTip;
    }

    /// <summary>
    /// Tunables for the final ground-contact stage. <see cref="bodyFollow"/> = 0 and
    /// <see cref="pitchFollow"/> = 0 reduce it to "feet only" (the plan's Level 1);
    /// raising them adds the pelvis vertical + pitch response (Level 2).
    /// </summary>
    [Serializable]
    public struct FootGroundingSettings
    {
        [Tooltip("Master toggle for the grounding stage.")]
        public bool enabled;

        [Tooltip("Extra contact bones sampled for the body settle IN ADDITION to the feet " +
                 "— e.g. the pelvis/hip (so the butt lands when sitting), sternum, chin, " +
                 "hocks. These are settle-only: the body moves to bring the lowest of ALL " +
                 "samples (feet + these) onto the ground; these bones get no IK of their own.")]
        public Transform[] extraContacts;

        [Tooltip("Use a single flat world ground plane (Ground Plane Y) instead of a " +
                 "per-foot raycast. Gives one perfectly-consistent level (no ray jitter); " +
                 "recommended for flat floors. Off = raycast the colliders (uneven ground).")]
        public bool useFixedPlane;
        [Tooltip("World Y of the flat ground plane, used when Use Fixed Plane is on. The " +
                 "FootGroundingStage overrides this each frame from its Ground Reference " +
                 "transform (e.g. the RootMotionAnchor's anchor) when one is set.")]
        public float groundPlaneY;

        [Tooltip("Layers treated as ground for the down-raycast (Use Fixed Plane off).")]
        public LayerMask groundMask;
        [Tooltip("Start the ray this far ABOVE the foot (m), so a foot already sunk " +
                 "below the surface still casts from above it. (Raycast mode.)")]
        public float castUp;
        [Tooltip("Max ray length below the start point (m). (Raycast mode.)")]
        public float maxDistance;
        [Tooltip("Lift the contact by this much (paw radius / sole thickness), m.")]
        public float footOffset;

        [Tooltip("Deadband for holding the level (m): the body only re-settles when the " +
                 "lowest sample moves more than this, so tiny per-frame wiggle doesn't " +
                 "bob the body — the contact level stays put.")]
        public float levelTolerance;

        [Range(0f, 1f)]
        [Tooltip("How much the whole body settles so the LOWEST foot meets the ground " +
                 "(drops a floating rig, raises a sunk one). 1 = body settles fully " +
                 "(recommended); 0 = body stays put and only the legs bend to reach down.")]
        public float bodyFollow;
        [Range(0f, 1f)]
        [Tooltip("How much the front/hind DIFFERENCE is absorbed as a body pitch. " +
                 "0 = no pitch (safest).")]
        public float pitchFollow;
        [Tooltip("Clamp on the body pitch (degrees). Pitch tilts the head too, so keep " +
                 "small (~8-12).")]
        public float maxPitchDegrees;

        [Tooltip("Also pull a slightly-floating planted foot DOWN onto the ground " +
                 "(within Snap Band). Off keeps swing feet untouched.")]
        public bool allowPullDownWhenPlanted;
        [Tooltip("Max gap a floating foot may be snapped down across (m).")]
        public float snapBand;

        [Tooltip("Time-smooth the ground height and body settle (seconds) so a grazing " +
                 "ray or jittering stance foot doesn't pop the rig. 0 = instant.")]
        public float heightDamp;
        [Tooltip("Draw the rays / contacts / body offset (Scene view, Play).")]
        public bool drawGizmos;

        /// <summary>Sensible starting values: a flat ground plane at world Y = 0.</summary>
        public static FootGroundingSettings Default => new FootGroundingSettings
        {
            enabled = true,
            extraContacts = null,
            useFixedPlane = true,
            groundPlaneY = 0f,
            groundMask = ~0,
            castUp = 0.5f,
            maxDistance = 1.5f,
            footOffset = 0.02f,
            levelTolerance = 0.02f,
            bodyFollow = 1f,
            pitchFollow = 0f,
            maxPitchDegrees = 12f,
            allowPullDownWhenPlanted = false,
            snapBand = 0.05f,
            heightDamp = 0.06f,
            drawGizmos = false,
        };
    }

    /// <summary>Per-driver persistent state for grounding: the smoothed ground height
    /// per foot and the smoothed body vertical/pitch, carried frame to frame so
    /// <see cref="FootGroundingSettings.heightDamp"/> can low-pass them.</summary>
    public sealed class FootGroundingState
    {
        public float[] smoothTargetY;
        public bool[] hasTargetY;
        public float smoothRise;
        public float smoothPitch;
        public bool hasBody;
        public readonly RaycastHit[] rayBuf = new RaycastHit[16];   // self-ignore raycast scratch

        public void Ensure(int legCount)
        {
            if (smoothTargetY == null || smoothTargetY.Length != legCount)
            {
                smoothTargetY = new float[legCount];
                hasTargetY = new bool[legCount];
                hasBody = false;
            }
        }
    }

    /// <summary>
    /// Final ground-contact stage (kinematic, no physics). After every solve, each foot
    /// measured against the ground — either a single flat world plane (one perfectly
    /// consistent level, recommended) or a self-ignoring per-sample raycast (uneven
    /// ground). "Samples" are the feet PLUS any extra contact bones (pelvis/sternum/chin),
    /// so the lowest part of the whole body — e.g. the butt when sitting — is what the
    /// body settles onto. The whole body is moved so that lowest sample rests on the
    /// ground (dropping a floating rig, raising a sunk one; higher samples stay above),
    /// with a tolerance deadband so the level holds steady. The front/hind difference can
    /// be absorbed as a small body pitch, and any FOOT still below the ground is then bent
    /// up by re-solving its IK (contacts have no IK — the body move alone grounds them,
    /// which is what folds the legs when the animal sits). See FOOT_GROUNDING_PLAN_kr.md.
    /// </summary>
    public static class FootGrounding
    {
        const float Eps = 1e-6f;

        /// <param name="root">Rig root to move for the body vertical/pitch response.</param>
        /// <param name="legs">Per-foot data + re-solve closures (driver-supplied).</param>
        /// <param name="up">World up (grounding axis). Usually <see cref="Vector3.up"/>.</param>
        /// <param name="bodyOrigin">Body centre (pitch pivot), from the driver's body frame.</param>
        /// <param name="bodyForward">Hips->shoulders (wheelbase + lateral axis source).</param>
        public static void Solve(Transform root, GroundLeg[] legs, Vector3 up,
                                 Vector3 bodyOrigin, Vector3 bodyForward,
                                 FootGroundingSettings s, FootGroundingState state, float dt)
        {
            if (root == null || legs == null)
                return;
            up = up.sqrMagnitude > Eps ? up.normalized : Vector3.up;

            // Samples = the IK-capable feet + the settle-only extra contacts (pelvis/
            // sternum/chin...). The body settles on the lowest of ALL of them, so a
            // non-foot part (e.g. the butt when sitting) can be what rests on the ground.
            Transform[] extra = s.extraContacts;
            int nLeg = legs.Length;
            int nExtra = extra != null ? extra.Length : 0;
            int n = nLeg + nExtra;
            if (n == 0)
                return;
            state.Ensure(n);

            var tips = new Transform[n];
            var isFront = new bool[n];
            var solve = new Action<Vector3>[n];
            for (int i = 0; i < nLeg; i++)
            {
                tips[i] = legs[i].tip;
                isFront[i] = legs[i].isFront;
                solve[i] = legs[i].solveToTip;   // feet can bend to the ground
            }
            for (int j = 0; j < nExtra; j++)
            {
                tips[nLeg + j] = extra[j];
                solve[nLeg + j] = null;          // contacts are settle-only (no IK)
            }

            // Step 1 — per-sample ground height along up. Fixed plane (one consistent
            // level) or, if that is off, a self-ignoring raycast per sample.
            var targetY = new float[n];
            var active = new bool[n];
            var sampleH = new float[n];
            float planeTop = s.groundPlaneY + s.footOffset;
            for (int i = 0; i < n; i++)
            {
                if (tips[i] == null)
                    continue;
                Vector3 p = tips[i].position;
                sampleH[i] = Vector3.Dot(p, up);

                if (s.useFixedPlane)
                {
                    targetY[i] = planeTop;   // rock-steady: same level every frame
                    active[i] = true;
                }
                else
                {
                    Vector3 origin = p + up * s.castUp;
                    if (RaycastGround(origin, -up, s.castUp + s.maxDistance, s, state.rayBuf,
                                      root, out Vector3 hitPoint))
                    {
                        float ty = Vector3.Dot(hitPoint, up) + s.footOffset;
                        if (s.heightDamp > 0f && state.hasTargetY[i] && dt > 0f)
                            ty = Mathf.Lerp(state.smoothTargetY[i], ty,
                                            Mathf.Clamp01(dt / s.heightDamp));
                        state.smoothTargetY[i] = ty;
                        state.hasTargetY[i] = true;
                        targetY[i] = ty;
                        active[i] = true;
                        if (s.drawGizmos)
                            Debug.DrawLine(origin, hitPoint, Color.yellow);
                    }
                    else
                    {
                        state.hasTargetY[i] = false;   // ray missed: no ground under this sample
                    }
                }
                if (active[i] && s.drawGizmos)
                    Debug.DrawRay(p + up * (targetY[i] - sampleH[i]), up * 0.05f,
                                  solve[i] != null ? Color.green : Color.blue);
            }

            // Step 2 — settle the body so the LOWEST sample meets the ground (signed:
            // drops a floating rig, raises a sunk one; higher samples stay above). A
            // deadband holds the level so sub-tolerance wiggle doesn't bob the body.
            int low = -1;
            float lowH = float.PositiveInfinity;
            for (int i = 0; i < n; i++)
            {
                if (active[i] && sampleH[i] < lowH)
                {
                    lowH = sampleH[i];
                    low = i;
                }
            }

            float v = low >= 0 ? (targetY[low] - sampleH[low]) * s.bodyFollow : 0f;
            if (state.hasBody && Mathf.Abs(v - state.smoothRise) < s.levelTolerance)
                v = state.smoothRise;   // hold the level (maintain it across small changes)

            // Step 3 — pitch (optional) from the front/hind penetration remaining after the
            // settle. Off by default (pitchFollow = 0). Only the feet drive the pitch.
            float theta = 0f;
            if (s.pitchFollow > 0f && low >= 0)
            {
                float frontPen = 0f, hindPen = 0f;
                bool anyFront = false, anyHind = false;
                for (int i = 0; i < nLeg; i++)
                {
                    if (!active[i])
                        continue;
                    float pen = targetY[i] - (sampleH[i] + v);   // penetration after the settle
                    if (isFront[i]) { frontPen = anyFront ? Mathf.Max(frontPen, pen) : pen; anyFront = true; }
                    else { hindPen = anyHind ? Mathf.Max(hindPen, pen) : pen; anyHind = true; }
                }
                float wheelbase = bodyForward.magnitude;
                if (anyFront && anyHind && wheelbase > 1e-4f)
                {
                    // Lateral = forward x up so a positive angle raises the front when it
                    // needs the greater rise (front/hind difference > 0).
                    theta = Mathf.Atan2((frontPen - hindPen) * s.pitchFollow, wheelbase)
                            * Mathf.Rad2Deg;
                    theta = Mathf.Clamp(theta, -s.maxPitchDegrees, s.maxPitchDegrees);
                }
            }

            if (s.heightDamp > 0f && state.hasBody && dt > 0f)
            {
                float t = Mathf.Clamp01(dt / s.heightDamp);
                v = Mathf.Lerp(state.smoothRise, v, t);
                theta = Mathf.Lerp(state.smoothPitch, theta, t);
            }
            state.smoothRise = v;
            state.smoothPitch = theta;
            state.hasBody = true;

            if (Mathf.Abs(v) > Eps)
                root.position += up * v;
            Vector3 pivot = bodyOrigin + up * v;   // body centre after the vertical move
            Vector3 lateral = Vector3.Cross(bodyForward, up);
            if (Mathf.Abs(theta) > 1e-4f && lateral.sqrMagnitude > Eps)
            {
                root.RotateAround(pivot, lateral, theta);
                if (s.drawGizmos)
                    Debug.DrawRay(pivot, lateral.normalized * 0.2f, Color.magenta);
            }
            if (s.drawGizmos && Mathf.Abs(v) > Eps)
                Debug.DrawRay(pivot, up * v, Color.cyan);

            // Step 4 — residual per-foot IK. The body move already did the common part;
            // bend only FEET still below the ground (contacts have no solve, so the body
            // move alone grounds them — which is what folds the legs when sitting).
            for (int i = 0; i < n; i++)
            {
                if (!active[i] || solve[i] == null || tips[i] == null)
                    continue;
                Vector3 p = tips[i].position;
                float h = Vector3.Dot(p, up);
                float pen = targetY[i] - h;
                bool up_ = pen > Eps;
                bool down = s.allowPullDownWhenPlanted && pen < 0f && -pen <= s.snapBand;
                if (!up_ && !down)
                    continue;
                Vector3 desired = p + up * (targetY[i] - h);   // XZ kept, height -> ground
                solve[i](desired);
            }
        }

        /// <summary>Nearest ground hit along the ray, skipping any collider that belongs
        /// to the rig itself (under <paramref name="ignoreRoot"/>). This keeps a foot ray
        /// from latching onto the animal's own body/limb colliders — a common cause of a
        /// wildly wrong "ground" height — even if the mask still includes those layers.</summary>
        static bool RaycastGround(Vector3 origin, Vector3 dir, float maxDist,
                                  FootGroundingSettings s, RaycastHit[] buf,
                                  Transform ignoreRoot, out Vector3 point)
        {
            int count = Physics.RaycastNonAlloc(origin, dir, buf, maxDist, s.groundMask,
                                                QueryTriggerInteraction.Ignore);
            float best = float.PositiveInfinity;
            point = default;
            bool got = false;
            for (int k = 0; k < count; k++)
            {
                Collider col = buf[k].collider;
                if (col == null)
                    continue;
                if (ignoreRoot != null &&
                    (col.transform == ignoreRoot || col.transform.IsChildOf(ignoreRoot)))
                    continue;
                if (buf[k].distance < best)
                {
                    best = buf[k].distance;
                    point = buf[k].point;
                    got = true;
                }
            }
            return got;
        }
    }
}
