using System;
using UnityEditor;
using UnityEngine;
using System.Collections.Generic;

public class Node
{
    public int gridX;
    public int gridY;
    public bool walkable;
    public Vector3 position;
    public int gcost;
    public int hcost;
    public int fcost { get { return gcost + hcost; } }
    public Node parent;
    public Node(int x, int y, bool _walkable, Vector3 _pos)
    {
        gridX = x;
        gridY = y;
        walkable = _walkable;
        position = _pos;
    }
}

public class Grid : MonoBehaviour
{
    Vector2 gridWorldSize;
    Node[,] grid;
    //
    float nodeDiameter;
    int gridSizeX;
    int gridSizeY;
    public List<Node> path;

    public void RebuildGrid(LayerMask unwalkableMask, Vector2 size, float nodeRadius)
    {
        gridWorldSize = new Vector2(size.x, size.y);
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
                grid[x, y] = new Node(x, y, walkable, pos);
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
    public Node GetNode(int x, int y)
    {
        if (grid == null)
        {
            return null;
        }
        return grid[x, y];
    }
    public List<Node> GetNeighbours(Node node)
    {
        List<Node> nodes = new List<Node>();
        for (int x = -1; x <= 1; x++)
            for (int y = -1; y <= 1; y++)
            {
                if (x == 0 && y == 0) continue;
                int checkX = node.gridX + x;
                int checkY = node.gridY + y;
                if (checkX < 0 || checkX >= gridSizeX || checkY < 0 || checkY >= gridSizeY) continue;
                nodes.Add(grid[checkX, checkY]);
            }
        return nodes;
    }

    private void OnDrawGizmos()
    {
        Gizmos.DrawCube(transform.position, new Vector3(gridWorldSize.x + 0.2f, 0.2f, gridWorldSize.y + 0.2f));
        if (grid != null)
        {
            foreach (var node in grid)
            {
                Gizmos.color = node.walkable ? Color.white : Color.red;
                if (path != null && path.Contains(node))
                {
                    Gizmos.color = Color.gray;
                }
                Gizmos.DrawCube(node.position, new Vector3(nodeDiameter - 0.1f, 0.4f, nodeDiameter - 0.1f));
            }
        }
        if (path != null)
        {
            Gizmos.color = Color.blue;
            for (int i = 1; i < path.Count; i++)
            {
                Gizmos.DrawLine(path[i - 1].position + Vector3.up, path[i].position + Vector3.up);
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
}