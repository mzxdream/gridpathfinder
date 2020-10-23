using UnityEngine;

public class MoveDef
{
    public int xsize = 1;
    public int zsize = 1;
    public float radius = 0.5f;
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

    public bool TestMoveSquare(Unit collider, Vector3 testMovePos, Vector3 testMoveDir, bool testTerrian = true, bool testObjects = true, bool centerOnly = false, float[] minSpeedModPtr = null, int[] maxBlockBitPtr = null)
    {
        return true;
    }
}