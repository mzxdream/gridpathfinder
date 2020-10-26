using UnityEngine;

public class PathMathUtils
{
    public static float SqrDistance2D(Vector3 pos1, Vector3 pos2)
    {
        var x = pos1.x - pos2.x;
        var z = pos1.z - pos2.z;
        return x * x + z * z;
    }
    public static float Square(float a)
    {
        return a * a;
    }
    public static float Mix(float v1, float v2, float a)
    {
        return v1 + (v2 - v1) * a;
    }
    public static Vector3 MixVec3(Vector3 v1, Vector3 v2, float a)
    {
        return v1 + (v2 - v1) * a;
    }
}