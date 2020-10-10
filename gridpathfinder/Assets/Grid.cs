using System;
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
    public void Start()
    {
        RebuildGrid();
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
        float percentX = Mathf.Clamp01((pos.x - transform.position.x + gridWorldSize.x / 2) / gridWorldSize.x);
        float percentY = Mathf.Clamp01((pos.y - transform.position.z + gridWorldSize.y / 2) / gridWorldSize.y);
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
    }
}