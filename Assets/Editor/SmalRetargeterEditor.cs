using System.Collections.Generic;
using PetDemo;
using UnityEditor;

[CustomEditor(typeof(SmalRetargeter))]
public class SmalRetargeterEditor : Editor
{
    SerializedProperty legMode;
    SerializedProperty applyGlobalMotion;

    // Fields that only affect the IK leg solve; hidden in FK mode. (ikSoftZone is
    // kept in both modes because it also shapes the FK stance-plant correction.)
    static readonly string[] IkOnly = { "maxLegRollDegrees" };

    // Axis masks that only matter when the global-motion overlay is on.
    static readonly string[] GlobalOnly = { "globalRotationAxes", "globalTranslationAxes" };

    void OnEnable()
    {
        legMode = serializedObject.FindProperty("legMode");
        applyGlobalMotion = serializedObject.FindProperty("applyGlobalMotion");
    }

    public override void OnInspectorGUI()
    {
        serializedObject.Update();

        var excluded = new List<string> { "m_Script" };
        if (legMode != null && legMode.enumValueIndex == (int)RetargetMode.FK)
            excluded.AddRange(IkOnly);
        if (applyGlobalMotion != null && !applyGlobalMotion.boolValue)
            excluded.AddRange(GlobalOnly);
        DrawPropertiesExcluding(serializedObject, excluded.ToArray());

        serializedObject.ApplyModifiedProperties();
    }
}
