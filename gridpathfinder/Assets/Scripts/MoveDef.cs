﻿using UnityEngine;

public class MoveDef
{
    public int xsize = 1;
    public int zsize = 1;
    public int xsizeh = 0;
    public int zsizeh = 0;
    //public float radius = 0.5f;
    public float speed = 0.2f; //速度
    public float rSpeed = 0.2f; //反向速度
    public float turnRate = 0.1f; //转向速度
    public bool turnInPlace = true; //是否可以原地转向
    public float turnInPlaceSpeedLimit = 0f; //对于具有turnInPlace = false的单位，它定义了将减慢到的最小速度（实际执行转弯的速度可以更高，具体取决于角度差和turnRate）。
    public float turnInPlaceAngleLimit = 0f; //对于turnInPlace = true的单位，定义转弯时开始制动的最大角度（度）。
    public bool collidable = true; //是否能被阻碍
    public bool collide = true; //是否是障碍
    public bool pushResistant = true; //该单元可以被其他单元推吗？启用它并不能消除所有推动，但有明显的改进。
    public bool avoidMobilesOnPath = true; //这些单位在路径转移时是否尝试避开其他移动单位？
    public float mobileBusyMult = 0.1f;
    public float mobileIdleMult = 0.35f;
    public float mobileMoveMult = 0.65f;
    public bool allowTerrainCollisions = true; //禁用此功能可改善对大（> 4 x 4）占地面积单元的处理
    public bool heatMapping = true;
    public float heatMod = 0.05f;
    public int heatProduced = 30;
    public float maxAcc = 0.1f;
    public float maxDec = 0.01f;

    public float CalcFootPrintMinExteriorRadius(float scale = 1.0f)
    {
        return ((Mathf.Sqrt(xsize * xsize + zsize * zsize) * 0.5f * Game.SQUARE_SIZE) * scale);
    }
    public float CalcFootPrintMaxInteriorRadius(float scale = 1.0f)
    {
        return ((Mathf.Max(xsize, zsize) * 0.5f * Game.SQUARE_SIZE) * scale);
    }

    public bool TestMoveSquare(Unit collider, Vector3 testMovePos, Vector3 testMoveDir, bool testTerrian = true, bool testObjects = true, bool centerOnly = false, float[] minSpeedModPtr = null, int[] maxBlockBitPtr = null)
    {
        return true;
    }
    public bool TestMoveSquareRange(Unit collider, Vector3 rangeMins, Vector3 rangeMaxs, Vector3 testMoveDir, bool testTerrain, bool testObjects, bool centerOnly, ref float minSpeedModPtr, ref int maxBlockBitPtr)
    {
        int xmin = (int)(rangeMins.x / Game.SQUARE_SIZE) - xsizeh * (1 - (centerOnly ? 1 : 0));
        int zmin = (int)(rangeMins.z / Game.SQUARE_SIZE) - zsizeh * (1 - (centerOnly ? 1 : 0));
        int xmax = (int)(rangeMaxs.x / Game.SQUARE_SIZE) + xsizeh * (1 - (centerOnly ? 1 : 0));
        int zmax = (int)(rangeMaxs.z / Game.SQUARE_SIZE) + zsizeh * (1 - (centerOnly ? 1 : 0));
        Vector3 testMoveDir2D = new Vector3(testMoveDir.x, 0, testMoveDir.z).normalized;
        float minSpeedMod = float.MaxValue;
        int maxBlockBit = (int)PathMathUtils.BlockTypes.BLOCK_NONE;
        bool retTestMove = true;

        for (int z = zmin; retTestMove && z <= zmax; z += 1)
        {
            for (int x = xmin; retTestMove && x <= xmax; x += 1)
            {
                float speedMod = 1.0f;
                minSpeedMod = Mathf.Min(minSpeedMod, speedMod);
                retTestMove &= (!testTerrain || (speedMod > 0.0f));
            }
        }
        // GetPosSpeedMod only checks *one* square of terrain
        // (heightmap/slopemap/typemap), not the blocking-map
        if (retTestMove)
        {
            var blockBits = PathMathUtils.RangeIsBlocked(this, xmin, xmax, zmin, zmax, collider);
            maxBlockBit |= blockBits;
            retTestMove &= (!testObjects || (blockBits & (int)PathMathUtils.BlockTypes.BLOCK_STRUCTURE) == 0);
        }
        // don't use std::min or |= because the ptr values might be garbage
        minSpeedModPtr = minSpeedMod;
        maxBlockBitPtr = maxBlockBit;
        return retTestMove;
    }
}