using System;
using UnityEngine;

namespace PetDemo
{
    /// <summary>
    /// DogPoseDriver variant that preserves the target rig's scale and maps each
    /// captured leg relative to its root, normalized to the target leg's bind length.
    /// </summary>
    [DefaultExecutionOrder(100)]
    public class NormalizedDogPoseDriver : DogPoseDriver
    {
        const float Eps = 1e-5f;

        float[] targetLegLengths;

        protected override void ApplyScale()
        {
            dogRoot.localScale = rootBindLocalScale;
        }

        protected override void OnClipLoaded()
        {
            targetLegLengths = new float[legs.Length];

            for (int index = 0; index < legs.Length; index++)
            {
                ResolvedLeg leg = legs[index];
                targetLegLengths[index] =
                    (leg.lower.position - leg.upper.position).magnitude +
                    (leg.end.position - leg.lower.position).magnitude;
                if (targetLegLengths[index] < Eps)
                    throw new InvalidOperationException(
                        $"NormalizedDogPoseDriver: target leg {index} has zero length");
            }
        }

        protected override void GetLegIKTarget(int index, ResolvedLeg leg,
                                               Vector3[] keypoints,
                                               out Vector3 target, out Vector3 pole)
        {
            Vector3 sourceRoot = keypoints[(int)leg.kneeJoint];
            Vector3 sourcePole = keypoints[(int)leg.poleJoint];
            Vector3 sourceFoot = keypoints[(int)leg.targetJoint];
            float sourceLength = (sourcePole - sourceRoot).magnitude +
                                 (sourceFoot - sourcePole).magnitude;
            if (sourceLength < Eps)
                throw new InvalidOperationException(
                    $"NormalizedDogPoseDriver: source leg '{leg.kneeJoint}' collapsed");

            float scale = targetLegLengths[index] / sourceLength;
            target = leg.upper.position +
                     (sourceFoot - sourceRoot) * scale;
            pole = leg.upper.position +
                   (sourcePole - sourceRoot) * scale;
        }
    }
}
