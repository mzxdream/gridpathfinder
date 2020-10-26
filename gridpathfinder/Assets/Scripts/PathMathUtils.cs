using System;
using UnityEngine;

public class PathMathUtils
{
    public enum BlockTypes
    {
        BLOCK_NONE = 0,
        BLOCK_MOVING = 1,
        BLOCK_MOBILE = 2,
        BLOCK_MOBILE_BUSY = 4,
        BLOCK_STRUCTURE = 8, //建筑
        BLOCK_IMPASSABLE = 24 // := 16 | BLOCK_STRUCTURE
    }
    public static float SqrDistance2D(Vector3 pos1, Vector3 pos2)
    {
        var x = pos1.x - pos2.x;
        var z = pos1.z - pos2.z;
        return x * x + z * z;
    }
    public static float Distance2D(Vector3 pos1, Vector3 pos2)
    {
        return Mathf.Sqrt(SqrDistance2D(pos1, pos2));
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
    public static float Sign(float v)
    {
        return v > 0f ? 1f : -1f;
    }
    public static float GetHeadingFromVectorF(float dx, float dz)
    {
        float h = 0.0f;
        if (dz != 0.0f)
        {
            // ensure a minimum distance between dz and 0 such that
            // sqr(dx/dz) never exceeds abs(num_limits<float>::max)
            float sz = dz * 2.0f - 1.0f;
            float d = dx / (dz + 0.000001f * sz);
            float dd = d * d;
            if (Mathf.Abs(d) > 1.0f)
            {
                h = (d > 0 ? 1.0f : -1.0f) * Mathf.PI * 0.5f - d / (dd + 0.28f);
            }
            else
            {
                h = d / (1.0f + 0.28f * dd);
            }
            h += ((Mathf.PI * ((dx > 0.0f ? 1.0f : 0.0f) * 2.0f - 1.0f)) * (dz < 0.0f ? 1.0f : 0.0f));
        }
        else
        {
            h = Mathf.PI * 0.5f * ((dx > 0.0f ? 1.0f : 0.0f) * 2.0f - 1.0f);
        }
        return h;
    }
    public static int GetHeadingFromVector(float dx, float dz)
    {
        float s = 32768 / Mathf.PI;
        float h = GetHeadingFromVectorF(dx, dz) * s;
        int ih = (int)h;
        ih += (ih == -32768 ? 1 : 0);
        ih %= 32768;
        return ih;
    }

    public static Vector3 GetVectorFromHeading(int heading)
    {
        int div = (32768 / 4096) * 2;
        int idx = heading / div + 4096 / 2;

        float ang = (idx - (4096 / 2)) * (Mathf.PI * 2) / 4096;
        float x = Mathf.Sin(ang);
        float y = Mathf.Cos(ang);
        return new Vector3(x, 0.0f, y);
    }
    public static int RangeIsBlocked(MoveDef moveDef, int xmin, int xmax, int zmin, int zmax, Unit collider)
    {
        return 0;
    }
    public static int SquareIsBlocked(MoveDef moveDef, int xSquare, int zSquare, Unit collider)
    {
        return 0;
    }
    public static int SquareIsBlocked(MoveDef moveDef, Vector3 pos, Unit collider)
    {
        return (SquareIsBlocked(moveDef, (int)pos.x / Game.SQUARE_SIZE, (int)pos.z / Game.SQUARE_SIZE, collider));
    }
    public static bool Epscmp0001(float a, float b)
    {
        return ((a == b) || (Mathf.Abs(a - b) <= (0.0001f * Mathf.Max(Mathf.Max(Mathf.Abs(a), Mathf.Abs(b)), 1.0f))));
    }
}