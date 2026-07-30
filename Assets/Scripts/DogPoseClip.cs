using System;
using UnityEngine;

namespace PetDemo
{
    /// <summary>
    /// One captured frame: 20 joint world positions (flat xyz*20) and per-joint
    /// confidence scores. Layout matches export_pose_positions.py exactly.
    /// </summary>
    [Serializable]
    public class DogPoseFrame
    {
        public float[] p; // 60 floats: x0,y0,z0, x1,y1,z1, ... x19,y19,z19
        public float[] s; // 20 confidence scores in [0, 1]
    }

    /// <summary>
    /// Deserialized InterPet4D clip. Positions are RAW capture-space values
    /// (Z-up, Y-forward, right-handed); the axis remap, scale and root
    /// placement live in <see cref="DogPoseDriver"/> so they stay tunable.
    /// </summary>
    [Serializable]
    public class DogPoseClip
    {
        public string clipName;
        public int frameCount;
        public int jointCount;
        public float fps;
        public string[] jointNames;
        public int[] parents;
        public DogPoseFrame[] frames;

        public static DogPoseClip FromJson(string json)
        {
            var clip = JsonUtility.FromJson<DogPoseClip>(json);
            if (clip == null || clip.frames == null || clip.frameCount <= 0)
                throw new ArgumentException("DogPoseClip: JSON missing frames");
            if (clip.jointCount != DogJoints.Count)
                throw new ArgumentException(
                    $"DogPoseClip: expected {DogJoints.Count} joints, got {clip.jointCount}");
            return clip;
        }

        /// <summary>World position of joint j in raw capture space for frame f.</summary>
        public Vector3 Position(int frame, int joint)
        {
            float[] p = frames[frame].p;
            int i = joint * 3;
            return new Vector3(p[i], p[i + 1], p[i + 2]);
        }

        public float Confidence(int frame, int joint) => frames[frame].s[joint];
    }
}
