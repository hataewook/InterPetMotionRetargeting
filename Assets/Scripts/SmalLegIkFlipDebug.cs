// =============================================================================
// DEBUG-ONLY diagnostic. Safe to delete: remove this single file (and its .meta)
// to fully strip the leg-IK flip visualization. It never modifies SmalRetargeter;
// it only reads its already-computed per-frame state via reflection, so deleting
// it leaves the retarget pipeline byte-for-byte unchanged.
//
// What it shows (front legs by default), each frame AFTER the IK solve:
//   white  : solved bone chain  upper -> lower(elbow) -> tip
//   cyan   : aim axis  upper -> target  (the axis OrientToPole twists about)
//   yellow : pole (Hint) marker + line from the elbow to the pole
//   green  : IK target goal marker
//   elbow sphere, colored by "pole agreement":
//       green = elbow bends toward the pole (correct side)
//       red   = elbow ended up on the WRONG side of the pole  <-- the flip
//   magenta ring at the elbow = near-straight "degeneracy zone" where the
//       bend-axis cross product and OrientToPole projection collapse.
//
// If the yellow/green markers glide smoothly while the elbow sphere flashes red
// and the white chain snaps, the flip is internal to TwoBoneIK (problem 1/2),
// not the input. Enable "Log Flips" to get a Console line at each flip.
// =============================================================================
using System.Reflection;
using UnityEngine;

namespace PetDemo
{
    [DefaultExecutionOrder(1000)] // run after SmalRetargeter's LateUpdate solve
    public class SmalLegIkFlipDebug : MonoBehaviour
    {
        [Header("Scope")]
        [Tooltip("Only visualize the front legs (name contains 'front', else index < 2).")]
        public bool frontOnly = true;
        [Tooltip("Master on/off for the gizmo drawing.")]
        public bool draw = true;

        [Header("Flip detection")]
        [Tooltip("Log a Console warning whenever a leg's bend crosses to the wrong side of its pole.")]
        public bool logFlips = true;
        [Tooltip("Interior elbow angle (deg) above which the leg is treated as near-straight " +
                 "(the degeneracy zone that triggers the flip). Drawn as a magenta ring.")]
        [Range(150f, 179f)] public float straightAngle = 165f;

        [Header("Appearance")]
        public float markerRadius = 0.03f;

        SmalRetargeter rt;
        FieldInfo fUpper, fLower, fTip, fPoles, fTargets, fLegs;
        int[] prevSide;         // per leg: last non-degenerate pole-agreement sign
        Vector3[] prevPoleDir;  // per leg: previous pole direction projected off the aim axis

        void Awake() => Bind();
        void OnValidate() => Bind();

        void Bind()
        {
            rt = GetComponent<SmalRetargeter>();
            if (rt == null)
                return;
            const BindingFlags F = BindingFlags.Instance | BindingFlags.NonPublic;
            var t = typeof(SmalRetargeter);
            fUpper = t.GetField("legUpper", F);
            fLower = t.GetField("legLower", F);
            fTip = t.GetField("legTip", F);
            fPoles = t.GetField("gizmoPoles", F);
            fTargets = t.GetField("gizmoTargets", F);
            fLegs = t.GetField("legs", F);
        }

        bool TryRead(out Transform[] up, out Transform[] lo, out Transform[] tip,
                     out Vector3[] poles, out Vector3[] targets, out System.Array legs)
        {
            up = lo = tip = null; poles = targets = null; legs = null;
            if (rt == null)
                Bind();
            if (rt == null || fUpper == null)
                return false;
            up = fUpper.GetValue(rt) as Transform[];
            lo = fLower.GetValue(rt) as Transform[];
            tip = fTip.GetValue(rt) as Transform[];
            poles = fPoles.GetValue(rt) as Vector3[];
            targets = fTargets.GetValue(rt) as Vector3[];
            legs = fLegs.GetValue(rt) as System.Array;
            return up != null && lo != null && tip != null && up.Length == lo.Length;
        }

        bool IsFront(System.Array legs, int i)
        {
            if (!frontOnly)
                return true;
            if (legs != null && i < legs.Length)
            {
                object leg = legs.GetValue(i);
                string n = leg?.GetType().GetField("name")?.GetValue(leg) as string;
                if (!string.IsNullOrEmpty(n))
                    return n.ToLowerInvariant().Contains("front");
            }
            return i < 2;
        }

        // >0: elbow bends toward the pole (correct). <0: flipped to the wrong side.
        // 0: degenerate (leg near-straight, so the side is undefined).
        static float PoleAgreement(Vector3 a, Vector3 elbow, Vector3 tip, Vector3 pole)
        {
            Vector3 aim = tip - a;
            if (aim.sqrMagnitude < 1e-8f)
                return 0f;
            aim.Normalize();
            Vector3 e = elbow - a; e -= Vector3.Dot(e, aim) * aim;
            Vector3 p = pole - a; p -= Vector3.Dot(p, aim) * aim;
            if (e.sqrMagnitude < 1e-10f || p.sqrMagnitude < 1e-10f)
                return 0f;
            return Vector3.Dot(e.normalized, p.normalized);
        }

        void LateUpdate()
        {
            if (!logFlips)
                return;
            if (!TryRead(out var up, out var lo, out var tip, out var poles, out _, out var legs))
                return;
            if (prevSide == null || prevSide.Length != up.Length)
                prevSide = new int[up.Length];
            if (prevPoleDir == null || prevPoleDir.Length != up.Length)
                prevPoleDir = new Vector3[up.Length];

            for (int i = 0; i < up.Length; i++)
            {
                if (up[i] == null || lo[i] == null || tip[i] == null || !IsFront(legs, i))
                    continue;
                Vector3 a = up[i].position, e = lo[i].position, c = tip[i].position;
                Vector3 pole = poles != null && i < poles.Length ? poles[i] : e;
                float ag = PoleAgreement(a, e, c, pole);

                // --- extra metrics that separate the sub-causes ------------------
                Vector3 aim = c - a;                                    // upper -> tip
                float interior = Vector3.Angle(a - e, c - e);          // ~180 => straight
                float aimVsLen = Vector3.Angle(e - a, c - a);          // small => aim axis ~ length axis
                Vector3 poleOffAim = aim.sqrMagnitude > 1e-8f
                    ? (pole - a) - Vector3.Dot(pole - a, aim.normalized) * aim.normalized
                    : Vector3.zero;
                float poleOff = poleOffAim.magnitude;                  // small => OrientToPole early-returns
                // Match the solver's weak/strong gate: below this the bend side is
                // deliberately held, so comparing the elbow to the (noisy) pole is
                // meaningless -- skip the flip verdict there instead of crying wolf.
                float denom = (c - a).magnitude * (pole - a).magnitude;
                float sinTheta = denom > 1e-6f ? Vector3.Cross(c - a, pole - a).magnitude / denom : 0f;
                bool poleStrong = sinTheta > 0.25f;
                bool poleInputCrossed =
                    prevPoleDir[i] != Vector3.zero && poleOffAim.sqrMagnitude > 1e-10f &&
                    Vector3.Dot(prevPoleDir[i], poleOffAim.normalized) < 0f;
                if (poleOffAim.sqrMagnitude > 1e-10f)
                    prevPoleDir[i] = poleOffAim.normalized;
                // ----------------------------------------------------------------

                // Only judge flips where the pole is trustworthy (strong signal);
                // the held/near-straight regime is expected to ignore the pole.
                if (!poleStrong)
                    continue;
                int side = ag > 0.02f ? 1 : ag < -0.02f ? -1 : 0;
                if (side != 0 && prevSide[i] != 0 && side != prevSide[i])
                    Debug.LogWarning(
                        $"[LegIkFlip] leg {i} -> {(side > 0 ? "pole side" : "WRONG side")} " +
                        $"ag={ag:F2} | interior={interior:F0}deg aimVsLen={aimVsLen:F0}deg " +
                        $"poleOff={poleOff:F3}m poleInputCrossed={poleInputCrossed}", this);
                if (side != 0)
                    prevSide[i] = side;
            }
        }

        void OnDrawGizmos()
        {
            if (!draw)
                return;
            if (!TryRead(out var up, out var lo, out var tip, out var poles, out var targets, out var legs))
                return;

            for (int i = 0; i < up.Length; i++)
            {
                if (up[i] == null || lo[i] == null || tip[i] == null || !IsFront(legs, i))
                    continue;

                Vector3 a = up[i].position, e = lo[i].position, c = tip[i].position;
                Vector3 pole = poles != null && i < poles.Length ? poles[i] : e;
                Vector3 target = targets != null && i < targets.Length ? targets[i] : c;

                Gizmos.color = Color.white;                 // solved bone chain
                Gizmos.DrawLine(a, e);
                Gizmos.DrawLine(e, c);

                Gizmos.color = Color.cyan;                  // aim axis upper->target
                Gizmos.DrawLine(a, target);

                Gizmos.color = Color.yellow;                // pole hint + elbow->pole
                Gizmos.DrawLine(e, pole);
                Gizmos.DrawWireSphere(pole, markerRadius * 0.7f);

                Gizmos.color = Color.green;                 // IK target goal
                Gizmos.DrawWireSphere(target, markerRadius * 0.7f);

                float ag = PoleAgreement(a, e, c, pole);    // elbow side vs pole
                Gizmos.color = ag >= 0f
                    ? Color.Lerp(Color.yellow, Color.green, ag)
                    : Color.Lerp(Color.yellow, Color.red, -ag);
                Gizmos.DrawSphere(e, markerRadius);

                float interior = Vector3.Angle(a - e, c - e); // near-straight ring
                if (interior >= straightAngle)
                {
                    Gizmos.color = Color.magenta;
                    Gizmos.DrawWireSphere(e, markerRadius * 1.8f);
                }
            }
        }
    }
}
