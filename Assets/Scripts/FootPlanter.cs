using UnityEngine;

namespace PetDemo
{
    /// <summary>
    /// Foot-contact lock (anti-skate). Detects a stance phase from the speed of a
    /// "detect" point (the raw source foot, in a space that does not move with the
    /// body) and, while in stance, freezes the "value" it hands back (the IK goal
    /// or the current tip position) at wherever it was when contact began. The
    /// hand-off is eased in and out over <c>attack</c> seconds so the foot does not
    /// pop when it plants or releases.
    ///
    /// Detection and value are separate on purpose: for a length-normalized rig the
    /// IK goal is expressed relative to the body, so it moves even when the real
    /// foot is planted — the raw foot keypoint is the honest stance signal.
    /// </summary>
    public class FootPlanter
    {
        bool locked;
        bool hasPrev;
        Vector3 prevDetect;
        Vector3 lockedValue;
        float weight;   // 0 = follow value freely, 1 = fully planted

        /// <param name="value">The position to freeze while planted (IK goal / tip).</param>
        /// <param name="detect">Body-independent stance signal (raw source foot).</param>
        /// <param name="lockSpeed">Enter stance when detect speed drops below this (m/s).</param>
        /// <param name="unlockSpeed">Leave stance when detect speed rises above this (m/s).</param>
        /// <param name="attack">Ease time for plant/release, seconds (0 = instant).</param>
        public Vector3 Filter(Vector3 value, Vector3 detect, float dt,
                              float lockSpeed, float unlockSpeed, float attack)
        {
            if (dt <= 0f)
                return hasPrev && weight > 0f ? Vector3.Lerp(value, lockedValue, weight) : value;

            float speed = hasPrev ? (detect - prevDetect).magnitude / dt : Mathf.Infinity;
            prevDetect = detect;
            hasPrev = true;

            if (locked)
            {
                if (speed > unlockSpeed)
                    locked = false;
            }
            else if (speed < lockSpeed)
            {
                locked = true;
                lockedValue = value;
            }

            float target = locked ? 1f : 0f;
            weight = attack > 0f ? Mathf.MoveTowards(weight, target, dt / attack) : target;
            if (weight <= 0f)
                return value;
            return Vector3.Lerp(value, lockedValue, weight);
        }

        public void Reset()
        {
            locked = false;
            hasPrev = false;
            weight = 0f;
        }
    }
}
