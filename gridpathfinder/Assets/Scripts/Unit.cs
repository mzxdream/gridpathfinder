using UnityEngine;

enum MoveState
{
    Done = 0,
    Active = 1,
    Failed = 2,
}
public class Unit
{
    public const float SquareSize = 1f;
    public const float WayPointRadius = SquareSize * 1.25f;
    Vector3 position;
    float radius;
    float maxSpeed;
    MoveState moveState;
    bool atGoal;
    bool atEndOfPath;
    Vector3 goalPos;
    float goalRadius;
    float extraRadius;
    Vector3 oldPos;
    Vector3 oldSlowUpdatePos;
    int numIdlingUpdates;
    int numIdlingSlowUpdates;
    float currWayPointDist;
    float prevWayPointDist;
    bool wantRepath;
    Vector3 velocity;
    float currentSpeed;
    float wantedSpeed;
    int pathID;
    Vector3 currWayPoint;
    Vector3 nextWayPoint;
    Vector3 realGoalPos;
    Vector3 tempGoalPos;
    public Unit(Vector3 position, float radius, float maxSpeed)
    {
        this.position = position;
        this.radius = radius;
        this.maxSpeed = maxSpeed;
        moveState = MoveState.Done;
    }
    public void StartMoving(Vector3 goalPos, float goalRadius = SquareSize)
    {
        this.goalPos = new Vector3(goalPos.x, 0, goalPos.z);
        this.goalRadius = goalRadius;
        this.extraRadius = TestMoveSquare(null, goalPos, Vector3.zero, true, true) ? 0 : Mathf.Max(0f, radius - goalRadius);
        atGoal = PathMathUtils.SqrDistance2D(this.position, goalPos) < (goalRadius + this.extraRadius) * (goalRadius + this.extraRadius);
        atEndOfPath = false;
        moveState = MoveState.Active;
        numIdlingUpdates = 0;
        numIdlingSlowUpdates = 0;
        currWayPointDist = 0f;
        prevWayPointDist = 0f;
        if (atGoal)
        {
            return;
        }
        ReRequestPath(true);
    }
    public void ReRequestPath(bool forceRequest = false)
    {
        if (forceRequest)
        {
            StopEngine();
            StartEngine();
            wantRepath = false;
            return;
        }
        wantRepath = true;
    }
    public int GetNewPath()
    {
        int newPathID = 0;
        if (PathMathUtils.SqrDistance2D(position, goalPos) <= (goalRadius + extraRadius) * (goalRadius + extraRadius))
        {
            return newPathID;
        }
        newPathID = PathManager.Instance().RequestPath(this, position, goalPos, goalRadius + extraRadius, true);
        if (newPathID != 0)
        {
            atGoal = false;
            atEndOfPath = false;
            currWayPoint = PathManager.Instance().NextWayPoint(this, newPathID, 0, position,     Mathf.Max(WayPointRadius, currentSpeed * 1.05f), true);
            nextWayPoint = PathManager.Instance().NextWayPoint(this, newPathID, 0, currWayPoint, Mathf.Max(WayPointRadius, currentSpeed * 1.05f), true);
            realGoalPos = goalPos;
            tempGoalPos = currWayPoint;
        }
        return newPathID;
    }
    public void StartEngine()
    {
        if (pathID == 0)
        {
            pathID = GetNewPath();
        }
        if (pathID != 0)
        {
            PathManager.Instance().UpdatePath(this, pathID);
        }
    }
    public void StopEngine(bool hardStop = false)
    {
        if (pathID != 0)
        {
            PathManager.Instance().DeletePath(pathID);
            pathID = 0;
        }
        if (hardStop)
        {
            velocity = Vector3.zero;
            currentSpeed = 0f;
        }
        wantedSpeed = 0f;
    }
    public bool TestMoveSquare(Unit collider, Vector3 testMovePos, Vector3 testMoveDir, bool testTerrian = true, bool testObjects = true, bool centerOnly = false)
    {
        return true;
    }
    public void Update()
    {
    }
    public void SlowUpdate()
    {
    }
}
