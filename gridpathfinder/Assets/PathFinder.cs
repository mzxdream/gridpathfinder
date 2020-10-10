using System;
using UnityEngine;

public class PathNode
{
    public int x;
    public int y;
    public bool walkable;
    public Vector3 pos;
    public PathNode(int _x, int _y, bool _walkable, Vector3 _pos)
    {
        x = _x;
        y = _y;
        walkable = _walkable;
        pos = _pos;
    }
}

public class PathFinder
{
    PathNode[,] nodes;
    public PathFinder(Node[,] grid)
    {
        nodes = new PathNode[grid.GetLength(0), grid.GetLength(1)];
        for (int x = 0; x < grid.GetLength(0); x++)
        {
            for (int y = 0; y < grid.GetLength(1); y++)
            {
                var n = grid[x, y];
                nodes[x, y] = new PathNode(x, y, n.walkable, n.position);
            }
        }
    }
    public void FindPath(Vector3 spos, Vector3 epos, Vector3[] path)
    {
    }
}
