using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using UnityEditorInternal;

public class PathFinderEditorWindow : EditorWindow
{
    LayerMask unwalkableMask = new LayerMask();
    Vector2 gridWorldSize = new Vector2();
    float nodeRadius;

    [MenuItem("Tools/寻路")]
    public static void ShowWindow()
    {
        EditorWindow.GetWindow(typeof(PathFinderEditorWindow), false, "寻路", true);
    }
    private static void DrawUILine(int thickness = 1, int padding = 10)
    {
        Color color = Color.gray;
        Rect r = EditorGUILayout.GetControlRect(GUILayout.Height(padding + thickness));
        r.height = thickness;
        r.y += padding / 2;
        r.x -= 2;
        r.width += 6;
        EditorGUI.DrawRect(r, color);
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
        DrawUILine();
        {
            GUILayout.BeginHorizontal();
            GUILayout.FlexibleSpace();
            GUILayout.Label("不可行走:");
            LayerMask tempMask = EditorGUILayout.MaskField(InternalEditorUtility.LayerMaskToConcatenatedLayersMask(unwalkableMask), InternalEditorUtility.layers);
            unwalkableMask = InternalEditorUtility.ConcatenatedLayersMaskToLayerMask(tempMask);
            GUILayout.EndHorizontal();
        }
        {
            GUILayout.BeginHorizontal();
            GUILayout.FlexibleSpace();
            GUILayout.Label("区域大小:");
            gridWorldSize = EditorGUILayout.Vector2Field("", gridWorldSize);
            GUILayout.EndHorizontal();
        }
        {
            GUILayout.BeginHorizontal();
            GUILayout.FlexibleSpace();
            GUILayout.Label("方格半径:");
            nodeRadius = EditorGUILayout.FloatField(nodeRadius);
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
        DrawUILine();
        GUILayout.EndVertical();
    }
}
