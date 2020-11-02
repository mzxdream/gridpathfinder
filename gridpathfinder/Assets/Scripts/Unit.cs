using System.Threading;
using UnityEngine;
using UnityEngine.Timeline;

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
    public Vector3 rightdir = -Vector3.right; //x-axis
    public Vector3 updir = Vector3.up; //y-axis



    //unit object
    public UnitDef unitDef;
    public MoveType moveType;
    public bool stunned = false; //晕眩
    public Vector3 midPos;
    public Vector3 aimPos;
    public Vector3 relMidPos;
    public Vector3 relAimPos;
    // if the updir is straight up or align to the ground vector
    public bool upright = true;
    public bool moving = false;
    public int allyteam = 0;
    public Unit()
    {
    }
    public void SetVelocityAndSpeed(Vector3 v)
    {
        speed = v;
        speedw = v.magnitude;
    }
    public void AddHeading(int deltaHeading, bool useGroundNormal, bool useObjectNormal)
    {
        SetHeading(heading + deltaHeading, useGroundNormal, useObjectNormal);
    }
    public void SetHeading(int worldHeading, bool useGroundNormal, bool useObjectNormal)
    {
        heading = worldHeading;
        UpdateDirVectors(useGroundNormal, useObjectNormal);
        UpdateMidAndAimPos();
    }
    void UpdateDirVectors(bool useGroundNormal, bool useObjectNormal)
    {
        updir = Vector3.up;
        frontdir = PathMathUtils.GetVectorFromHeading(heading);
        rightdir = Vector3.Cross(frontdir, updir).normalized;
        frontdir = Vector3.Cross(updir, rightdir);
    }
    void UpdateMidAndAimPos()
    {
        midPos = GetObjectSpacePos(relMidPos);
        aimPos = GetObjectSpacePos(relAimPos);
    }
    public Vector3 GetObjectSpacePos(Vector3 p)
    {
        return (pos + (frontdir * p.z + (rightdir * p.x) + (updir * p.y)));
    }
}
