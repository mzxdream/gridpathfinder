using UnityEngine;

public enum SpeedModMults
{
    IDLE = 0,
    BUSY = 1,
    MOVE = 2,
    NUM = 3,
}

public class MoveDef
{
    //MoveDef
    public int xsize = 2; // 只能是2、4、6、8等2的n倍
    public int zsize = 2; // 只能是2、4、6、8等2的n倍
    public int xsizeh = 1; // xsize / 2
    public int zsizeh = 1; // zsize / 2
    public bool avoidMobilesOnPath = true; //这些单位在路径转移时是否尝试避开其他移动单位？
    public float[] speedModMults = new float[] { 1.0f, 1.0f, 1.0f, 0.0f }; //对于不同状态的单位，避开的参数
    public bool allowTerrainCollisions = true; //禁用此功能可改善对大（> 4 x 4）占地面积单元的处理
    public float CalcFootPrintMinExteriorRadius(float scale = 1.0f)
    {
        return ((Mathf.Sqrt(xsize * xsize + zsize * zsize) * 0.5f * Game.SQUARE_SIZE) * scale);
    }
    public float CalcFootPrintMaxInteriorRadius(float scale = 1.0f)
    {
        return ((Mathf.Max(xsize, zsize) * 0.5f * Game.SQUARE_SIZE) * scale);
    }
    public bool TestMoveSquare(Unit collider, int xTestMoveSqr, int zTestMoveSqr, Vector3 testMoveDir, bool testTerrain = true, bool testObjects = true, bool centerOnly = false)
    {
        return true;
    }
    public bool TestMoveSquare(Unit collider, Vector3 testMovePos, Vector3 testMoveDir, bool testTerrain = true, bool testObjects = true, bool centerOnly = false)
    {
        int xTestMoveSqr = (int)testMovePos.x / Game.SQUARE_SIZE;
        int zTestMoveSqr = (int)testMovePos.z / Game.SQUARE_SIZE;
        return TestMoveSquare(collider, xTestMoveSqr, zTestMoveSqr, testMoveDir, testTerrain, testObjects, centerOnly);
    }
}