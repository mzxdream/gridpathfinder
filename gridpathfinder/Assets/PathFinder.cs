using System;
using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;

public class PathFinder : MonoBehaviour
{
    Grid grid;
    void RetracePath(Node startNode, Node endNode)
    {
        List<Node> path = new List<Node>();
        Node currentNode = endNode;
        while (currentNode != startNode)
        {
            path.Add(currentNode);
            currentNode = currentNode.parent;
        }
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
                if (!neighbour.walkable || closeSet.Contains(neighbour))
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
    public void FindPath()
    {
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
