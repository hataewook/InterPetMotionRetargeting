using System;

namespace PetDemo
{
    /// <summary>InterPet4D MMPose AnimalPose-3D keypoint indices.</summary>
    public enum DogKeypoint
    {
        L_Eye = 0, R_Eye = 1, L_EarBase = 2, R_EarBase = 3, Nose = 4,
        Throat = 5, TailBase = 6, Withers = 7,
        L_F_Knee = 8, R_F_Knee = 9, L_B_Knee = 10, R_B_Knee = 11,
        L_F_Elbow = 12, R_F_Elbow = 13, L_B_Elbow = 14, R_B_Elbow = 15,
        L_F_Paw = 16, R_F_Paw = 17, L_B_Paw = 18, R_B_Paw = 19,
    }

    public enum DogRigPreset
    {
        DogRejoint,
        GermanShepherd,
        Custom,
    }

    /// <summary>Target-bone names for the supported rigs, indexed by DogKeypoint.</summary>
    public static class DogJoints
    {
        public static readonly int Count = Enum.GetValues(typeof(DogKeypoint)).Length;

        static readonly string[] DogRejointBones =
        {
            "Bone.032", "Bone.033", null, null, null,
            "Bone.002", "Bone.022", "Bone.001",
            "Bone.009", "Bone.010", "Bone.017", "Bone.018",
            "Bone.011", "Bone.012", "Bone.019", "Bone.020",
            "Bone.011_end", "Bone.012_end", "Bone.019_end", "Bone.020_end",
        };

        static readonly string[] GermanShepherdBones =
        {
            null, null,
            "DEF-Border-collie_ear01.L", "DEF-Border-collie_ear01.R", null,
            "DEF-spine.009", "DEF-spine.004", "DEF-spine.006",
            "DEF-front_shin.L", "DEF-front_shin.R", "DEF-shin.L", "DEF-shin.R",
            "DEF-front_foot.L", "DEF-front_foot.R", "DEF-foot.L", "DEF-foot.R",
            "DEF-front_toe.L", "DEF-front_toe.R", "DEF-toe.L", "DEF-toe.R",
        };

        public static string[] BoneNames(DogRigPreset preset)
        {
            switch (preset)
            {
                case DogRigPreset.DogRejoint:
                    return (string[])DogRejointBones.Clone();
                case DogRigPreset.GermanShepherd:
                    return (string[])GermanShepherdBones.Clone();
                case DogRigPreset.Custom:
                    throw new InvalidOperationException(
                        "Custom mapping has no preset bone-name table");
                default:
                    throw new ArgumentOutOfRangeException(nameof(preset), preset, null);
            }
        }
    }
}
