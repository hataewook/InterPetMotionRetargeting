using UnityEngine;

namespace PetDemo
{
    /// <summary>
    /// A driver (SmalRetargeter / DogPoseDriver) that places its rig root from the
    /// captured body each frame and can report where the body is. RootMotionAnchor
    /// reads this to re-base the root motion onto an anchor.
    /// </summary>
    public interface IBodyFrameProvider
    {
        /// <summary>The rig root the driver moves for root motion.</summary>
        Transform BodyRoot { get; }

        /// <summary>Current body centre and facing (world space), measured from the
        /// posed rig. Returns false until the rig is resolved / posed.</summary>
        bool TryGetBodyFrame(out Vector3 origin, out Vector3 forward);
    }

    /// <summary>
    /// Re-bases the root motion produced by a <see cref="IBodyFrameProvider"/> driver
    /// (SmalRetargeter or *DogPoseDriver) onto an assigned anchor: the body's first
    /// pose is moved so its centre sits at the anchor and its facing points along the
    /// anchor's +Z (forward). Only yaw (rotation about world up) is applied, so the
    /// body never tips. The anchor is treated as the ground position — the captured
    /// vertical motion is preserved and lifted so the capture's y = 0 ground sits at
    /// the anchor's height.
    ///
    /// A single rigid transform is computed once from the first posed frame and then
    /// applied to the root every frame, so the captured relative motion (walking,
    /// turning) is preserved exactly — only its start point and heading are re-based.
    ///
    /// Put this on the SAME GameObject as the driver. It runs after the driver (high
    /// execution order + LateUpdate) so it reads the root the driver just placed. For
    /// SmalRetargeter, enable its Apply Global Motion so there is root motion to
    /// re-base.
    /// </summary>
    [DefaultExecutionOrder(1000)]
    public class RootMotionAnchor : MonoBehaviour
    {
        [Tooltip("Its position becomes the start of the root motion (the ground), and " +
                 "its +Z (forward) becomes the facing of the first pose.")]
        [SerializeField] Transform anchor;

        [Tooltip("Lift the captured motion so its ground (y = 0) sits at the anchor's " +
                 "height. Off: keep the captured world Y unchanged (align X/Z only).")]
        [SerializeField] bool matchAnchorHeight = true;

        [Tooltip("Draw the anchor position and facing in the Scene view.")]
        [SerializeField] bool drawGizmos = true;

        const float Eps = 1e-8f;

        IBodyFrameProvider provider;
        bool calibrated;
        Quaternion yaw = Quaternion.identity;   // world-up rotation: first-pose facing -> anchor forward
        Vector3 offset;                          // translation after yaw

        void Awake()
        {
            provider = GetComponent<IBodyFrameProvider>();
            if (provider == null)
                throw new MissingComponentException(
                    "RootMotionAnchor: needs a driver implementing IBodyFrameProvider " +
                    "(SmalRetargeter or a DogPoseDriver) on the same GameObject");
            if (anchor == null)
                throw new MissingReferenceException("RootMotionAnchor: no anchor assigned");
        }

        /// <summary>Re-measure the alignment from the current first pose next frame
        /// (e.g. after changing the clip or moving the anchor in Play).</summary>
        [ContextMenu("Recalibrate")]
        public void Recalibrate() => calibrated = false;

        void LateUpdate()
        {
            Transform root = provider.BodyRoot;
            if (root == null)
                return;

            if (!calibrated)
            {
                if (!provider.TryGetBodyFrame(out Vector3 c0, out Vector3 fwd0))
                    return;   // rig not posed yet; try again next frame

                // Yaw only: flatten both facings to the world XZ plane so the rotation
                // is purely about world up and never tips the body.
                yaw = Quaternion.FromToRotation(Flatten(fwd0), Flatten(anchor.forward));

                // Place the body centre at the anchor horizontally; lift so the
                // capture's ground meets the anchor height (yaw about Y keeps Y).
                Vector3 rc0 = yaw * c0;
                offset = new Vector3(
                    anchor.position.x - rc0.x,
                    matchAnchorHeight ? anchor.position.y : 0f,
                    anchor.position.z - rc0.z);
                calibrated = true;
            }

            root.SetPositionAndRotation(yaw * root.position + offset, yaw * root.rotation);
        }

        static Vector3 Flatten(Vector3 v)
        {
            v.y = 0f;
            return v.sqrMagnitude < Eps ? Vector3.forward : v;
        }

        void OnDrawGizmosSelected()
        {
            if (!drawGizmos || anchor == null)
                return;
            Gizmos.color = Color.cyan;
            Gizmos.DrawSphere(anchor.position, 0.03f);
            Gizmos.DrawLine(anchor.position, anchor.position + anchor.forward * 0.5f);
        }
    }
}
