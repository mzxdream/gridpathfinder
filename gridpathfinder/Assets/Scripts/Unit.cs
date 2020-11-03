using UnityEngine;

public class Unit
{
    //world object
    public Vector3 pos; //对象的中心最低点
    public Vector3 speed; //当前的速度，elmos/frame
    public float speedw; //|speed|
    public float radius;//障碍物半径
    //solid object
    public float mass = 1e5f; //质量 默认unitdef->mass
    public int heading = 0; //朝向 和frontdir一致
    public int allyteam = 0;
    public MoveDef moveDef;
    public Vector3 frontdir = Vector3.forward; //z-axis
    public Vector3 rightdir = -Vector3.right; //x-axis == frontdir.cross(updir)
    public Vector3 updir = Vector3.up; //y-axis 简化下总是向上，实际应该根据地形的坡度来算
    //unit object
    public UnitDef unitDef;
    public MoveType moveType;
    public Unit()
    {
    }
    public void SetVelocityAndSpeed(Vector3 v)
    {
        speed = v;
        speedw = v.magnitude;
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