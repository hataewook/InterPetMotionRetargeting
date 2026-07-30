using UnityEngine;

namespace PetDemo
{
    /// <summary>World axes a root-align step is allowed to act on.</summary>
    [System.Flags]
    public enum RootAxis
    {
        None = 0,
        X = 1,
        Y = 2,
        Z = 4,
        All = X | Y | Z,
    }

    /// <summary>
    /// Restricts a root-follow translation or rotation to selected world axes. Used by
    /// the pose drivers so the body can, say, yaw to follow the captured heading and
    /// track it horizontally without also rolling/pitching or bobbing from noise in the
    /// alignment.
    /// </summary>
    public static class RootAxisMask
    {
        /// <summary>Keep only the selected world-axis components of a translation.</summary>
        public static Vector3 Vector(Vector3 v, RootAxis axes)
        {
            if ((axes & RootAxis.X) == 0) v.x = 0f;
            if ((axes & RootAxis.Y) == 0) v.y = 0f;
            if ((axes & RootAxis.Z) == 0) v.z = 0f;
            return v;
        }

        /// <summary>
        /// Restrict a world-space rotation to the selected axes by zeroing the
        /// unselected components of its Euler decomposition. Euler is order-dependent,
        /// so combinations are approximate; single-axis masks — the common case — are
        /// exact.
        /// </summary>
        public static Quaternion Rotation(Quaternion q, RootAxis axes)
        {
            if (axes == RootAxis.All)
                return q;
            if (axes == RootAxis.None)
                return Quaternion.identity;
            Vector3 e = q.eulerAngles;
            if ((axes & RootAxis.X) == 0) e.x = 0f;
            if ((axes & RootAxis.Y) == 0) e.y = 0f;
            if ((axes & RootAxis.Z) == 0) e.z = 0f;
            return Quaternion.Euler(e);
        }
    }
}
