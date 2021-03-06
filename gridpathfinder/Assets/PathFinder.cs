﻿using System;
using System.Collections.Generic;
using System.Diagnostics;
using TMPro;
using UnityEngine;

public class PathFinder : MonoBehaviour
{
    Grid grid;
    int unitSize;
    bool CheckNodeWalkable(Node snode, Node enode)
    {
        int signX = enode.gridX > snode.gridX ? 1 : -1;
        int signY = enode.gridY > snode.gridY ? 1 : -1;
        if (snode.gridX != enode.gridX)
        {
            if (snode.gridY != enode.gridY)
            {
                int ex = snode.gridX + signX * unitSize;
                int ey = snode.gridY + signY * unitSize;
                if (!grid.GetNode(ex, ey).walkable)
                {
                    return false;
                }
                int n = Math.Max(1, 2 * unitSize - 2);
                for (int i = n; i > 0; i--)
                {
                    if (!grid.GetNode(ex - i * signX, ey).walkable)
                    {
                        return false;
                    }
                }
                for (int i = n; i > 0; i--)
                {
                    if (!grid.GetNode(ex, ey - i * signY).walkable)
                    {
                        return false;
                    }
                }
            }
            else
            {
                int ex = snode.gridX + unitSize * signX;
                int ey1 = snode.gridY - (unitSize - 1);
                int ey2 = snode.gridY + (unitSize - 1);
                for (int i = ey1; i <= ey2; i++)
                {
                    var n = grid.GetNode(ex, i);
                    if (!n.walkable)
                    {
                        return false;
                    }
                }
            }
        }
        else
        {
            int ex1 = snode.gridX - (unitSize - 1);
            int ex2 = snode.gridX + (unitSize - 1);
            int ey = snode.gridY + unitSize * signY;
            for (int i = ex1; i <= ex2; i++)
            {
                var n = grid.GetNode(i, ey);
                if (!n.walkable)
                {
                    return false;
                }
            }
        }
        return true;
    }
    bool CheckCrossWalkable(Node snode, Node enode)
    {
        var px = snode.gridX;
        var py = snode.gridY;
        var qx = enode.gridX;
        var qy = enode.gridY;
        var dx = qx - px;
        var dy = qy - py;
        var nx = Math.Abs(dx);
        var ny = Math.Abs(dy);
        var signX = dx > 0 ? 1 : -1;
        var signY = dy > 0 ? 1 : -1;
        for (int ix = 0, iy = 0; ix < nx || iy < ny;)
        {
            if ((1 + 2 * ix) * ny == (1 + 2 * iy) * nx) //对角线
            {
                px += signX;
                py += signY;
                ix++;
                iy++;
                if (!CheckNodeWalkable(grid.GetNode(px - signX, py - signY), grid.GetNode(px, py)))
                {
                    return false;
                }
            }
            else if ((1 + 2 * ix) * ny < (1 + 2 * iy) * nx) //水平
            {
                px += signX;
                ix++;
                if (!CheckNodeWalkable(grid.GetNode(px - signX, py), grid.GetNode(px, py)))
                {
                    return false;
                }
            }
            else //垂直
            {
                py += signY;
                iy++;
                if (!CheckNodeWalkable(grid.GetNode(px, py - signY), grid.GetNode(px, py)))
                {
                    return false;
                }
            }
        }
        return true;
    }
    List<Node> SmoothPath(List<Node> path)
    {
        List<Node> waypoints = new List<Node>();
        if (path.Count < 2)
        {
            return waypoints;
        }
        //去除同一直线的点
        waypoints.Add(path[0]);
        Vector2 oldDirection = new Vector2(path[0].gridX - path[1].gridX, path[0].gridY - path[1].gridY);
        for (int i = 2; i < path.Count; i++)
        {
            Vector2 newDirection = new Vector2(path[i - 1].gridX - path[i].gridX, path[i - 1].gridY - path[i].gridY);
            if (oldDirection != newDirection)
            {
                waypoints.Add(path[i - 1]);
                oldDirection = newDirection;
            }
        }
        waypoints.Add(path[path.Count - 1]);
        //去除拐点
        for (int i = waypoints.Count - 1; i > 1; i--)
        {
            for (int j = 0; j < i - 1; j++)
            {
                if (CheckCrossWalkable(waypoints[i], waypoints[j]))
                {
                    for (int k = i - 1; k > j; k--)
                    {
                        waypoints.RemoveAt(k);
                    }
                    i = j + 1;
                    break;
                }
            }
        }
        return waypoints;
    }
    void RetracePath(Node startNode, Node endNode)
    {
        List<Node> path = new List<Node>();
        Node currentNode = endNode;
        while (currentNode != startNode)
        {
            path.Add(currentNode);
            currentNode = currentNode.parent;
        }
        path.Add(startNode);
        path = SmoothPath(path);
        path.Reverse();
        grid.path = path;
        UnityEngine.Debug.Log("find path finished");
    }
    void FindPath(Node startNode, Node endNode)
    {
        List<Node> openSet = new List<Node>();
        HashSet<Node> closeSet = new HashSet<Node>();
        openSet.Add(startNode);
        while (openSet.Count > 0)
        {
            Node node = openSet[0];
            for (int i = 1; i < openSet.Count; i++)
            {
                if (openSet[i].fcost < node.fcost || (openSet[i].fcost == node.fcost && openSet[i].hcost < node.hcost))
                {
                    node = openSet[i];
                }
            }
            openSet.Remove(node);
            closeSet.Add(node);
            if (node == endNode)
            {
                RetracePath(startNode, endNode);
                return;
            }
            foreach (var neighbour in grid.GetNeighbours(node))
            {
                if (!CheckNodeWalkable(node, neighbour) || closeSet.Contains(neighbour))
                {
                    continue;
                }
                int newCost = node.gcost + GetDistance(node, neighbour);
                if (!openSet.Contains(neighbour) || newCost < neighbour.gcost)
                {
                    neighbour.gcost = newCost;
                    neighbour.hcost = GetDistance(neighbour, endNode);
                    neighbour.parent = node;
                    if (!openSet.Contains(neighbour))
                    {
                        openSet.Add(neighbour);
                    }
                }
            }
        }
    }
    int GetDistance(Node snode, Node enode)
    {
        int x = Mathf.Abs(snode.gridX - enode.gridX);
        int y = Mathf.Abs(snode.gridY - enode.gridY);
        if (x > y)
            return 14 * y + 10 * (x - y);
        return 14 * x + 10 * (y - x);
    }
    public void FindPath(int usize)
    {
        unitSize = usize;
        grid = GetComponent<Grid>();
        if (!grid)
        {
            return;
        }
        grid.path = new List<Node>();
        var target = GameObject.FindGameObjectWithTag("Target");
        if (target == null)
        {
            return;
        }
        var endNode = grid.GetNodeFromPos(target.transform.position);
        if (endNode == null)
        {
            return;
        }
        var players = GameObject.FindGameObjectsWithTag("Player");
        foreach (var p in players)
        {
            var startNode = grid.GetNodeFromPos(p.transform.position);
            if (startNode != null)
            {
                FindPath(startNode, endNode);
            }
        }
    }
}
