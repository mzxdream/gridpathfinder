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
    [SerializeField] private GameObject walkablePrefab;
    [SerializeField] private GameObject unwalkablePrefab;
    PathGrid grid;
    GameObject[,] gridObjs;
    // Start is called before the first frame update
    void Start()
    {
        grid = new PathGrid(gameObject.transform.position, width, height, cellSize);
        gridObjs = new GameObject[width, height];
        for (int x = 0; x < grid.Width; x++)
        {
            for (int y = 0; y < grid.Height; y++)
            {
                var node = grid.GetNode(x, y);
                var go = Instantiate(walkablePrefab, null, false);
                go.transform.position = node.position;
                go.transform.localScale = go.transform.localScale * cellSize;
                gridObjs[x, y] = go;
            }
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetMouseButtonDown(0))
        {
            var ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            if (Physics.Raycast(ray, out RaycastHit hit, Mathf.Infinity))
            {
                var pos = hit.point;
                var node = grid.GetNodeFromPos(pos);
                node.walkable = !node.walkable;
                Destroy(gridObjs[node.x, node.y]);
                if (node.walkable)
                {
                    var go = Instantiate(walkablePrefab, null, false);
                    go.transform.position = node.position;
                    go.transform.localScale = go.transform.localScale * cellSize;
                    gridObjs[node.x, node.y] = go;
                }
                else
                {
                    var go = Instantiate(unwalkablePrefab, null, false);
                    go.transform.position = node.position;
                    go.transform.localScale = go.transform.localScale * cellSize;
                    gridObjs[node.x, node.y] = go;
                }
            }
        }
    }
}
