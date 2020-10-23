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
    public Unit()
    {
    }
    public void SetVelocityAndSpeed(Vector3 v)
    {
        speed = v;
        speedw = v.magnitude;
    }
}
