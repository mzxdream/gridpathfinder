using UnityEngine;

public class Unit
{
    //world object
    public Vector3 pos; //对象的中心最低点
    public Vector3 speed; //当前的速度，elmos/frame
    public float speedw; //|speed|
    public float radius;//障碍物半径
    //solid object
    public int heading = 0; //朝向 和frontdir一致
    public int allyteam = 0;
    public MoveDef moveDef;
    public Vector3 frontdir = Vector3.forward; //z-axis
    public Vector3 rightdir = -Vector3.right; //x-axis == frontdir.cross(updir)
    public Vector3 updir = Vector3.up; //y-axis 简化下总是向上，实际应该根据地形的坡度来算
    //unit object
    public UnitDef unitDef;
    public MoveType moveType;

    public int mapSquare; //地图上位置
    public int mapPosX;
    public int mapPosZ;
    public Vector3 groundBlockPos;
    public bool blockEnemyPushing;
    public bool IsMoving { get; set; }
    public Unit(Vector3 pos)
    {
        this.pos = pos;
        this.speed = Vector3.zero;
        this.speedw = 0f;
        this.radius = 16f;
        this.moveDef = new MoveDef();
        this.unitDef = new UnitDef();
        this.moveType = new MoveType(this);
    }
    public void Block()
    {
    }
    public void SetVelocityAndSpeed(Vector3 v)
    {
        speed = v;
        speedw = v.magnitude;
    }
    public void Move(Vector3 v, bool relative)
    {
        var dv = relative ? v : v - pos;
        pos += dv;
    }
    public void AddHeading(int deltaHeading)
    {
        SetHeading(heading + deltaHeading);
    }
    public void SetHeading(int worldHeading)
    {
        heading = worldHeading;
        updir = Vector3.up;
        frontdir = PathMathUtils.GetVectorFromHeading(heading);
        rightdir = Vector3.Cross(frontdir, updir).normalized;
    }
}