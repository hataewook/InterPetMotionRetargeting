using System;
using UnityEngine;

namespace PetDemo
{
    /// <summary>
    /// Plays a SMAL clip with NO imported FBX rig: it builds its own lightweight joint
    /// skeleton (empty GameObjects named "SMAL_joint_00".."SMAL_joint_34") from the clip's
    /// rest skeleton and poses it each frame. Together with a clip exported by
    /// <c>Python/export_smal_unity_clip_rigfree.py</c> — which bakes the SMAL->Unity
    /// coordinate conversion (rotation + handedness + scale) straight into the data — this
    /// gives a fully rig-free SMAL source: no dog FBX, no runtime calibration.
    ///
    /// The generated skeleton is a drop-in source for the existing <see cref="SmalRetargeter"/>:
    /// point that component's <c>Smal Source</c> at this GameObject and it finds the joints
    /// as descendants by name. (Copy-rotation still falls back to the frame-0 neutral, since
    /// there is no <see cref="SmalMotionPlayer"/>; its per-axis remap needs re-tuning as with
    /// any non-FBX source.)
    ///
    /// How the pose is reproduced without bone rest orientations: the skeleton is built with
    /// every joint's rest LOCAL rotation set to identity and its local position set to the
    /// rest offset from its parent, so the rig's rest world positions equal the clip's
    /// (Unity-space) <c>restJoints</c>. Each frame the clip's per-joint world rotation delta
    /// is written as the joint's world rotation (rest = identity), and Unity's transform
    /// hierarchy turns that into the correct joint positions — the same forward kinematics
    /// <see cref="SmalMotionPlayer"/> gets from the FBX hierarchy, minus the FBX. The captured
    /// global body transform is applied to the skeleton container.
    ///
    /// Runs early (negative execution order) so the skeleton exists and is posed at frame 0
    /// before a retargeter's Awake/Start resolves and reads it.
    /// </summary>
    [DefaultExecutionOrder(-100)]
    public class SmalMotionPlayerRigFree : MonoBehaviour
    {
        [Header("Clip")]
        [Tooltip("JSON produced by Python/export_smal_unity_clip_rigfree.py (baked into " +
                 "Unity space). A raw export_smal_unity_clip.py clip will play mirrored.")]
        [SerializeField] TextAsset clipJson;

        [Header("Playback")]
        [Tooltip("0 = use the clip's own fps.")]
        [SerializeField] float fpsOverride;
        [SerializeField] bool loop = true;
        [Tooltip("Apply the captured global body motion (turn / translate / scale) to the " +
                 "skeleton. Disable to keep it planted and play only the articulation.")]
        [SerializeField] bool applyGlobalMotion = true;

        [Header("Debug")]
        [SerializeField] bool logInfo = true;

        [Serializable]
        class Clip
        {
            public string name;
            public int fps;
            public int frameCount;
            public int jointCount;
            public string boneNamePrefix;
            public string space;       // "unity" for a rig-free (baked) clip
            public int[] parents;
            public float[] restJoints; // jointCount * 3
            public float[] deltas;     // frameCount * jointCount * 4  (x,y,z,w)
            public float[] rootRot;    // frameCount * 4               (x,y,z,w)
            public float[] rootPos;    // frameCount * 3
            public float[] rootScale;  // frameCount
        }

        Clip clip;
        Transform skelRoot;            // container the global body transform is applied to
        Transform[] bones;             // [joint] generated joint transform
        Vector3[] restPos;             // [joint] Unity-space rest position (from the clip)
        float time;

        void Awake()
        {
            if (clipJson == null)
                throw new MissingReferenceException("SmalMotionPlayerRigFree: no clip JSON assigned");

            clip = JsonUtility.FromJson<Clip>(clipJson.text);
            if (clip == null || clip.frameCount <= 0 || clip.jointCount <= 0)
                throw new InvalidOperationException("SmalMotionPlayerRigFree: clip JSON failed to parse");
            if (clip.parents == null || clip.parents.Length < clip.jointCount
                || clip.restJoints == null || clip.restJoints.Length < clip.jointCount * 3)
                throw new InvalidOperationException(
                    "SmalMotionPlayerRigFree: clip is missing parents / restJoints (re-export the clip)");
            if (clip.space != "unity")
                Debug.LogWarning("SmalMotionPlayerRigFree: clip is not marked space=\"unity\" — it " +
                    "was probably made with export_smal_unity_clip.py (raw SMAL space) and will play " +
                    "mirrored. Use export_smal_unity_clip_rigfree.py.");

            BuildSkeleton();
            Pose(0, 0, 0f);

            if (logInfo)
                Debug.Log($"SmalMotionPlayerRigFree: built rig-free skeleton for '{clip.name}' " +
                          $"({clip.jointCount} joints, {clip.frameCount} frames, fps={clip.fps})");
        }

        void BuildSkeleton()
        {
            int n = clip.jointCount;
            string prefix = string.IsNullOrEmpty(clip.boneNamePrefix) ? "SMAL_joint_" : clip.boneNamePrefix;

            restPos = new Vector3[n];
            for (int j = 0; j < n; j++)
                restPos[j] = new Vector3(
                    clip.restJoints[j * 3 + 0], clip.restJoints[j * 3 + 1], clip.restJoints[j * 3 + 2]);

            skelRoot = new GameObject(name + "_SmalSkeleton").transform;
            skelRoot.SetParent(transform, false);
            skelRoot.localPosition = Vector3.zero;
            skelRoot.localRotation = Quaternion.identity;
            skelRoot.localScale = Vector3.one;

            bones = new Transform[n];
            // Parents precede children in SMAL joint order, so a single forward pass can
            // parent each joint under an already-created parent.
            for (int j = 0; j < n; j++)
            {
                var go = new GameObject($"{prefix}{j:00}");
                int p = clip.parents[j];
                Transform parent = p < 0 ? skelRoot : bones[p];
                go.transform.SetParent(parent, false);
                // Rest: identity local rotation, local position = offset from the parent's
                // rest position, so the rig's rest world positions equal the clip restJoints.
                Vector3 parentRest = p < 0 ? Vector3.zero : restPos[p];
                go.transform.localRotation = Quaternion.identity;
                go.transform.localPosition = restPos[j] - parentRest;
                go.transform.localScale = Vector3.one;
                bones[j] = go.transform;
            }
        }

        void Update()
        {
            float fps = fpsOverride > 0f ? fpsOverride : clip.fps;
            time += Time.deltaTime;
            float framePos = time * fps;
            int f = Mathf.FloorToInt(framePos);
            float frac = framePos - f;
            int next;
            if (loop)
            {
                f = ((f % clip.frameCount) + clip.frameCount) % clip.frameCount;
                next = (f + 1) % clip.frameCount;
            }
            else
            {
                f = Mathf.Min(f, clip.frameCount - 1);
                next = Mathf.Min(f + 1, clip.frameCount - 1);
                if (f == next)
                    frac = 0f;
            }
            Pose(f, next, frac);
        }

        void Pose(int f, int next, float frac)
        {
            // Global body transform on the container (Unity space already, so no
            // conjugation). Everything below rides this: rotation feeds the joints' world
            // rotations, translation/scale flow through the hierarchy into the positions.
            if (applyGlobalMotion)
            {
                var rot = new Quaternion(clip.rootRot[f * 4 + 0], clip.rootRot[f * 4 + 1],
                                         clip.rootRot[f * 4 + 2], clip.rootRot[f * 4 + 3]);
                var pos = new Vector3(clip.rootPos[f * 3 + 0], clip.rootPos[f * 3 + 1],
                                      clip.rootPos[f * 3 + 2]);
                float s = clip.rootScale[f];
                if (frac > 0f)
                {
                    var rotN = new Quaternion(clip.rootRot[next * 4 + 0], clip.rootRot[next * 4 + 1],
                                              clip.rootRot[next * 4 + 2], clip.rootRot[next * 4 + 3]);
                    var posN = new Vector3(clip.rootPos[next * 3 + 0], clip.rootPos[next * 3 + 1],
                                           clip.rootPos[next * 3 + 2]);
                    rot = Quaternion.Slerp(rot, rotN, frac);
                    pos = Vector3.Lerp(pos, posN, frac);
                    s = Mathf.Lerp(s, clip.rootScale[next], frac);
                }
                skelRoot.localPosition = pos;
                skelRoot.localRotation = rot;
                skelRoot.localScale = new Vector3(s, s, s);
            }
            else
            {
                skelRoot.localPosition = Vector3.zero;
                skelRoot.localRotation = Quaternion.identity;
                skelRoot.localScale = Vector3.one;
            }

            Quaternion baseRot = skelRoot.rotation;
            int baseF = f * clip.jointCount * 4;
            int baseN = next * clip.jointCount * 4;
            for (int j = 0; j < clip.jointCount; j++)   // parents precede children in SMAL order
            {
                int i = baseF + j * 4;
                var delta = new Quaternion(
                    clip.deltas[i + 0], clip.deltas[i + 1], clip.deltas[i + 2], clip.deltas[i + 3]);
                if (frac > 0f)
                {
                    int m = baseN + j * 4;
                    var deltaNext = new Quaternion(
                        clip.deltas[m + 0], clip.deltas[m + 1], clip.deltas[m + 2], clip.deltas[m + 3]);
                    delta = Quaternion.Slerp(delta, deltaNext, frac);
                }
                // Rest world rotation is identity, so the joint's world rotation is just the
                // container rotation times its own world delta. Set parents first (done by
                // joint order) so each child reads a finalized parent when Unity recomputes
                // its position.
                bones[j].rotation = baseRot * delta;
            }
        }

        /// <summary>The generated skeleton root (the joints are its descendants). Assign a
        /// retargeter's source to THIS GameObject (or this root); both find the joints by
        /// name as descendants.</summary>
        public Transform SkeletonRoot => skelRoot;

        /// <summary>Rest local rotation of a joint. The rig-free skeleton is built with
        /// identity rest local rotations, so this is always identity — provided for parity
        /// with copy-rotation consumers that ask for a neutral reference.</summary>
        public bool TryGetRestLocalRotation(int joint, out Quaternion rest)
        {
            rest = Quaternion.identity;
            return bones != null && joint >= 0 && joint < bones.Length;
        }
    }
}
