using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEditorInternal;
using UnityEngine;

public class Testing : MonoBehaviour
{
    [SerializeField] private int width;
    [SerializeField] private int height;
    [SerializeField] private float cellSize;
    [SerializeField] private GameObject blockPrefab;
    PathGrid grid;
    GameObject[,] blocks;
    // Start is called before the first frame update
    void Start()
    {
        grid = new PathGrid(gameObject.transform.position, width, height, cellSize);
        blocks = new GameObject[width, height];
        for (int x = 0; x < grid.Width; x++)
        {
            var node = grid.GetNode(x, 0);
            node.walkable = false;
            blocks[x, 0] = CreateBlock(node.position);
            node = grid.GetNode(x, grid.Height - 1);
            node.walkable = false;
            blocks[x, grid.Height - 1] = CreateBlock(node.position);
        }
        for (int y = 1; y < grid.Height - 1; y++)
        {
            var node = grid.GetNode(0, y);
            node.walkable = false;
            blocks[0, y] = CreateBlock(node.position);
            node = grid.GetNode(grid.Width - 1, y);
            node.walkable = false;
            blocks[grid.Width - 1, y] = CreateBlock(node.position);
        }
    }
    GameObject CreateBlock(Vector3 pos)
    {
        var go = Instantiate(blockPrefab, null, false);
        go.transform.position = pos;
        go.transform.localScale = go.transform.localScale * cellSize;
        return go;
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetMouseButtonDown(1))
        {
            var ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            if (Physics.Raycast(ray, out RaycastHit hit, Mathf.Infinity))
            {
                var pos = hit.point;
                var node = grid.GetNodeFromPos(pos);
                if (node.x != 0 && node.x != grid.Width - 1 && node.y != 0 && node.y != grid.Height - 1)
                {
                    node.walkable = !node.walkable;
                    if (!node.walkable)
                    {
                        blocks[node.x, node.y] = CreateBlock(node.position);
                    }
                    else
                    {
                        Destroy(blocks[node.x, node.y]);
                        blocks[node.x, node.y] = null;
                    }
                }
            }
        }
    }
}
