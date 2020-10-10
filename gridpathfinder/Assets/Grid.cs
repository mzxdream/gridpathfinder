using System;
using UnityEditor;
using UnityEngine;

public class Node
{
    public bool walkable;
    public Vector3 position;
    public Node(bool _walkable, Vector3 _pos)
    {
        walkable = _walkable;
        position = _pos;
    }
}

public class Grid : MonoBehaviour
{
    public LayerMask unwalkableMask;
    public Vector2 gridWorldSize;
    public float nodeRadius;
    Node[,] grid;
    //
    float nodeDiameter;
    int gridSizeX;
    int gridSizeY;

    bool targetChange;
    long lastTime;
    long curTime;
    public void Start()
    {
        RebuildGrid();
        SceneView.FocusWindowIfItsOpen(typeof(SceneView));
        UnityEditor.SceneView.beforeSceneGui += OnSceneFunc;
        targetChange = false;
        curTime = lastTime = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
    }
    public void RebuildGrid()
    {
        nodeDiameter = nodeRadius * 2;
        gridSizeX = Mathf.RoundToInt(gridWorldSize.x / nodeDiameter);
        gridSizeY = Mathf.RoundToInt(gridWorldSize.y / nodeDiameter);
        grid = new Node[gridSizeX, gridSizeY];
        Vector3 worldBottomLeft = transform.position - Vector3.right * gridWorldSize.x / 2 - Vector3.forward * gridWorldSize.y / 2;
        for (int x = 0; x < gridSizeX; x++)
        {
            for (int y = 0; y < gridSizeY; y++)
            {
                Vector3 pos = worldBottomLeft + Vector3.right * (x * nodeDiameter + nodeRadius) + Vector3.forward * (y * nodeDiameter + nodeRadius);
                bool walkable = !Physics.CheckSphere(pos, nodeRadius, unwalkableMask);
                grid[x, y] = new Node(walkable, pos);
            }
        }
    }

    public Node GetNodeFromPos(Vector3 pos)
    {
        if (grid == null)
        {
            return null;
        }
        float percentX = Mathf.Clamp01((pos.x - transform.position.x + gridWorldSize.x / 2) / gridWorldSize.x);
        float percentY = Mathf.Clamp01((pos.z - transform.position.z + gridWorldSize.y / 2) / gridWorldSize.y);
        int x = Mathf.RoundToInt((gridSizeX - 1) * percentX);
        int y = Mathf.RoundToInt((gridSizeY - 1) * percentY);
        return grid[x, y];
    }

    private void OnDrawGizmos()
    {
        Gizmos.DrawCube(transform.position, new Vector3(gridWorldSize.x + 0.2f, 0.2f, gridWorldSize.y + 0.2f));
        if (grid != null)
        {
            foreach (var node in grid)
            {
                Gizmos.color = node.walkable ? Color.white : Color.red;
                Gizmos.DrawCube(node.position, new Vector3(nodeDiameter - 0.1f, 0.4f, nodeDiameter - 0.1f));
            }
        }
        var players = GameObject.FindGameObjectsWithTag("Player");
        foreach (var p in players)
        {
            var node = GetNodeFromPos(p.transform.position);
            if (node != null)
            {
                Gizmos.color = Color.green;
                Gizmos.DrawCube(node.position, new Vector3(nodeDiameter - 0.1f, 0.6f, nodeDiameter - 0.1f));
            }
        }
        var target = GameObject.FindGameObjectWithTag("Target");
        if (target != null)
        {
            var node = GetNodeFromPos(target.transform.position);
            if (node != null)
            {
                Gizmos.color = Color.black;
                Gizmos.DrawCube(node.position, new Vector3(nodeDiameter - 0.1f, 0.5f, nodeDiameter - 0.1f));
            }
        }
    }
    void OnSceneFunc(SceneView sceneView)
    {
        Event e = Event.current;
        if (e.type == EventType.MouseDown && e.button == 2)
        {
            Vector3 mousePos = e.mousePosition;
            float ppp = EditorGUIUtility.pixelsPerPoint;
            mousePos.y = sceneView.camera.pixelHeight - mousePos.y * ppp;
            mousePos.x *= ppp;

            Ray ray = sceneView.camera.ScreenPointToRay(mousePos);
            RaycastHit hit;
            if (Physics.Raycast(ray, out hit))
            {
                var target = GameObject.FindGameObjectWithTag("Target");
                if (target != null)
                {
                    target.transform.position = hit.point;
                    targetChange = true;
                }
            }
            e.Use();
        }
    }
    private void Update()
    {
        if (targetChange)
        {
            //var players = GameObject.FindGameObjectsWithTag("Player");
            //foreach (var p in players)
            //{
            //    var node = GetNodeFromPos(p.transform.position);
            //    if (node != null)
            //    {
            //        Gizmos.color = Color.green;
            //        Gizmos.DrawCube(node.position, new Vector3(nodeDiameter - 0.1f, 0.6f, nodeDiameter - 0.1f));
            //    }
            //}
        }
        curTime = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        while (lastTime < curTime)
        {
            //
            lastTime += 33;
        }
    }
}