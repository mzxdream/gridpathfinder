using UnityEngine;

public class PathMathUtils
{
    public static float SqrDistance2D(Vector3 pos1, Vector3 pos2)
    {
        var x = pos1.x - pos2.x;
        var z = pos1.z - pos2.z;
        return x * x + z * z;
    }
}