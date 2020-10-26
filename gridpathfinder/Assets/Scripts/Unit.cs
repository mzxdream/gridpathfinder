using System.Threading;
using UnityEngine;
using UnityEngine.Timeline;

public class Unit
{
    public Vector3 pos;
    public Vector3 speed;
    public float speedw;
    public MoveDef moveDef;
    public MoveType moveType;
    public bool stunned = false;
    public int heading = 0;
    public Vector3 frontdir = Vector3.forward;
    public Vector3 rightdir = -Vector3.right;
    public Vector3 updir = Vector3.up;
    public Vector3 midPos;
    public Vector3 aimPos;
    public Vector3 relMidPos;
    public Vector3 relAimPos;
    // if the updir is straight up or align to the ground vector
    public bool upright = true;
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
