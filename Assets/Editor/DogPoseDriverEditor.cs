using System.Collections.Generic;
using PetDemo;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(DogPoseDriver), true)]
public class DogPoseDriverEditor : Editor
{
    SerializedProperty mappingPreset;
    SerializedProperty jointBones;
    SerializedProperty dogRoot;
    SerializedProperty legMode;
    SerializedProperty alignRootToTrunk;

    // Fields that only affect the IK leg solve; hidden in FK mode.
    static readonly string[] IkOnly =
    {
        "limitIkLegRoll", "maxIkLegRollDegrees",
        "leftFrontPoleWeight", "rightFrontPoleWeight",
        "leftBackPoleWeight", "rightBackPoleWeight",
    };

    void OnEnable()
    {
        mappingPreset = serializedObject.FindProperty("mappingPreset");
        jointBones = serializedObject.FindProperty("jointBones");
        dogRoot = serializedObject.FindProperty("dogRoot");
        legMode = serializedObject.FindProperty("legMode");
        alignRootToTrunk = serializedObject.FindProperty("alignRootToTrunk");
    }

    public override void OnInspectorGUI()
    {
        serializedObject.Update();

        var excluded = new List<string> { "m_Script", "jointBones" };
        if (legMode != null && legMode.enumValueIndex == (int)LegMode.FK)
            excluded.AddRange(IkOnly);
        // The trunk-align rotation axis mask only matters when trunk-align is on.
        if (alignRootToTrunk != null && !alignRootToTrunk.boolValue)
            excluded.Add("alignRotationAxes");
        DrawPropertiesExcluding(serializedObject, excluded.ToArray());

        EditorGUILayout.Space();
        EditorGUILayout.LabelField("3D Keypoint -> Target Bone", EditorStyles.boldLabel);
        EditorGUILayout.HelpBox(
            "Apply a preset, then override any row with a Transform from the target rig. " +
            "Eye, nose, and ear rows may remain empty unless used by Head Aim. " +
            "EarBase alone does not contain enough information to animate ear rotation.",
            MessageType.Info);

        using (new EditorGUILayout.HorizontalScope())
        {
            if (GUILayout.Button("Apply Dog_rejoint"))
                ApplyPreset(DogRigPreset.DogRejoint);
            if (GUILayout.Button("Apply P_GermanShepherd"))
                ApplyPreset(DogRigPreset.GermanShepherd);
        }

        if (jointBones.arraySize != DogJoints.Count)
            jointBones.arraySize = DogJoints.Count;
        EditorGUI.BeginChangeCheck();
        for (int index = 0; index < DogJoints.Count; index++)
        {
            var keypoint = (DogKeypoint)index;
            EditorGUILayout.PropertyField(
                jointBones.GetArrayElementAtIndex(index),
                new GUIContent($"{index:00}  {keypoint}"));
        }
        if (EditorGUI.EndChangeCheck())
            mappingPreset.enumValueIndex = (int)DogRigPreset.Custom;

        Transform root = dogRoot.objectReferenceValue as Transform;
        if (root == null)
            root = ((DogPoseDriver)target).transform;
        Animator animator = root.GetComponentInChildren<Animator>(true);
        if (animator != null && animator.enabled &&
            animator.runtimeAnimatorController != null)
            EditorGUILayout.HelpBox(
                $"Disable Animator '{animator.name}' before Play. It would compete " +
                "with DogPoseDriver for the same bones.",
                MessageType.Warning);

        EditorGUILayout.Space();
        if (GUILayout.Button("Validate Mapping"))
        {
            serializedObject.ApplyModifiedProperties();
            ((DogPoseDriver)target).ValidateMapping();
            Debug.Log("DogPoseDriver mapping is valid", target);
        }
        serializedObject.ApplyModifiedProperties();
    }

    void ApplyPreset(DogRigPreset preset)
    {
        Undo.RecordObject(target, $"Apply {preset} DogPose mapping");
        mappingPreset.enumValueIndex = (int)preset;
        serializedObject.ApplyModifiedProperties();
        var driver = (DogPoseDriver)target;
        driver.ApplySelectedMappingPreset();
        EditorUtility.SetDirty(driver);
        serializedObject.Update();
    }
}
