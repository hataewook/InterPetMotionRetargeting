using UnityEngine;

namespace PetDemo
{
    /// <summary>
    /// Thin stage that runs the ground-contact solve after the driver and (optional)
    /// <see cref="RootMotionAnchor"/>. Put it on the SAME GameObject as an
    /// <see cref="IGroundable"/> driver (SmalRetargeter or a DogPoseDriver). Order 2000
    /// &gt; anchor's 1000 &gt; driver, so it reads the final world pose and raycasts the
    /// feet against the real ground colliders. See FOOT_GROUNDING_PLAN_kr.md.
    ///
    /// The scene needs ground colliders on the <see cref="FootGroundingSettings.groundMask"/>
    /// layers. When used with a RootMotionAnchor, see that anchor's Match Anchor Height
    /// tooltip for how the two split the vertical.
    /// </summary>
    [DefaultExecutionOrder(2000)]
    public class FootGroundingStage : MonoBehaviour
    {
        [SerializeField] FootGroundingSettings settings = FootGroundingSettings.Default;

        [Tooltip("Optional: take the fixed ground plane's height from this transform's world " +
                 "Y each frame, instead of the Settings' Ground Plane Y. Drag your " +
                 "RootMotionAnchor's anchor (or any floor marker) so the floor has a single " +
                 "source of truth. Auto-filled from a RootMotionAnchor on this object if left " +
                 "empty. Only used when Use Fixed Plane is on.")]
        [SerializeField] Transform groundReference;

        IGroundable groundable;
        readonly FootGroundingState state = new FootGroundingState();

        void Awake()
        {
            groundable = GetComponent<IGroundable>();
            if (groundable == null)
                throw new MissingComponentException(
                    "FootGroundingStage: needs an IGroundable driver (SmalRetargeter or a " +
                    "DogPoseDriver) on the same GameObject");

            // Default the ground plane's source to the RootMotionAnchor's own anchor, so
            // "where the floor is" is defined once (on the anchor) and grounding follows it.
            if (groundReference == null)
            {
                RootMotionAnchor anchor = GetComponent<RootMotionAnchor>();
                if (anchor != null)
                    groundReference = anchor.GroundAnchor;
            }
        }

        void LateUpdate()
        {
            if (!settings.enabled)
                return;

            FootGroundingSettings s = settings;   // struct copy: override the plane per frame
            if (s.useFixedPlane && groundReference != null)
                s.groundPlaneY = groundReference.position.y;
            groundable.ApplyFootGrounding(s, state);
        }
    }
}
