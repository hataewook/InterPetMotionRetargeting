using UnityEngine;

namespace PetDemo
{
    /// <summary>
    /// One-Euro low-pass filter for a Vector3 signal. Adapts its cutoff to the
    /// signal speed: heavy smoothing when the value is slow (kills jitter) and
    /// almost no smoothing when it moves fast (no lag on quick motion). This is
    /// the right filter for noisy motion-capture keypoints because it removes the
    /// per-frame tremor without smearing real, fast articulation.
    ///
    /// Reference: Casiez, Roussel, Vogel, "1€ Filter" (CHI 2012).
    /// </summary>
    public class OneEuroFilterVector3
    {
        readonly float minCutoff;   // Hz: base cutoff at rest (lower = smoother)
        readonly float beta;        // speed coefficient (higher = less lag when fast)
        readonly float dCutoff;     // Hz: cutoff for the derivative estimate

        bool has;
        Vector3 xPrev;
        Vector3 dxPrev;

        public OneEuroFilterVector3(float minCutoff = 1f, float beta = 0.02f, float dCutoff = 1f)
        {
            this.minCutoff = Mathf.Max(minCutoff, 1e-4f);
            this.beta = Mathf.Max(beta, 0f);
            this.dCutoff = Mathf.Max(dCutoff, 1e-4f);
        }

        static float Alpha(float cutoff, float dt)
        {
            float tau = 1f / (2f * Mathf.PI * cutoff);
            return 1f / (1f + tau / dt);
        }

        public Vector3 Filter(Vector3 x, float dt)
        {
            if (dt <= 0f)
                return has ? xPrev : x;
            if (!has)
            {
                has = true;
                xPrev = x;
                dxPrev = Vector3.zero;
                return x;
            }

            Vector3 dx = (x - xPrev) / dt;
            float aD = Alpha(dCutoff, dt);
            Vector3 dxHat = Vector3.Lerp(dxPrev, dx, aD);

            float cutoff = minCutoff + beta * dxHat.magnitude;
            float a = Alpha(cutoff, dt);
            Vector3 xHat = Vector3.Lerp(xPrev, x, a);

            xPrev = xHat;
            dxPrev = dxHat;
            return xHat;
        }

        public void Reset() => has = false;
    }
}
