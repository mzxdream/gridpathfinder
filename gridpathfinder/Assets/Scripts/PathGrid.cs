using System;
using UnityEditor;
using UnityEngine;
using System.Collections.Generic;
using System.Net.Http.Headers;

public class PathNode
{
    public Vector3 position;
    public int x;
    public int y;
    public bool walkable;
    public int gCost;
    public int hCost;
    public PathNode parent;
    public PathNode(Vector3 position, int x, int y, bool walkable)
    {
        this.position = position;
        this.x = x;
        this.y = y;
        this.walkable = walkable;
    }
    public int FCost { get { return gCost + hCost; } }
}

public class PathGrid
{
    Vector3 originPosition;
    int width;
    int height;
    float cellSize;
    PathNode[,] nodes;
    public PathGrid(Vector3 originPosition, int width, int height, float cellSize)
    {
        this.originPosition = originPosition;
        this.width = width;
        this.height = height;
        this.cellSize = cellSize;
        nodes = new PathNode[width, height];
        for (int x = 0; x < nodes.GetLength(0); x++)
        {
            for (int y = 0; y < nodes.GetLength(1); y++)
            {
                Vector3 pos = originPosition + new Vector3(x + 0.5f, 0f, y + 0.5f) * cellSize;
                nodes[x, y] = new PathNode(pos, x, y, true);
            }
        }
    }
    public int Width { get => width; }
    public int Height { get => height; }
    public PathNode GetNode(int x, int y)
    {
        return nodes[x, y];
    }
    public PathNode GetNodeFromPos(Vector3 worldPosition)
    {
        int x = Mathf.Clamp(Mathf.FloorToInt((worldPosition.x - originPosition.x) / cellSize), 0, width - 1);
        int y = Mathf.Clamp(Mathf.FloorToInt((worldPosition.z - originPosition.z) / cellSize), 0, height - 1);
        return nodes[x, y];
    }
}