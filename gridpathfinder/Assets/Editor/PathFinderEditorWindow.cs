using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

public class PathFinderEditorWindow : EditorWindow
{
    [MenuItem("Tools/寻路")]
    public static void ShowWindow()
    {
        EditorWindow.GetWindow(typeof(PathFinderEditorWindow), false, "寻路", true);
    }
    private void OnGUI()
    {
        GUILayout.BeginVertical(GUILayout.Width(300));
        EditorGUILayout.Space();
        EditorGUILayout.Space();
        EditorGUILayout.Space();
        {
            GUILayout.BeginHorizontal();
            GUILayout.FlexibleSpace();
            GUILayout.Label("寻路");
            GUILayout.FlexibleSpace();
            GUILayout.EndHorizontal();
        }
        {
            GUILayout.BeginHorizontal();
            GUILayout.FlexibleSpace();
            if (GUILayout.Button("生成格子", GUILayout.Width(70)))
            {
                var obj = UnityEngine.Object.FindObjectOfType<Grid>();
                if (obj != null)
                {
                    obj.GetComponent<Grid>().RebuildGrid();
                }
            }
            GUILayout.FlexibleSpace();
            GUILayout.EndHorizontal();
        }
        GUILayout.EndVertical();
    }
}
