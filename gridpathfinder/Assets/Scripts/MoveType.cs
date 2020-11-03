using System.Globalization;
using UnityEngine;

public class MoveType
{
    public const float WAYPOINT_RADIUS = 1.25f * Game.SQUARE_SIZE;
    //AMoveType
    public Unit owner;
    public Vector3 goalPos;
    public Vector3 oldPos; // owner position at last Update()
    public Vector3 oldSlowUpdatePos; // owner position at last SlowUpdate()
    public enum ProgressState { Done, Active, Failed };
    public ProgressState progressState = ProgressState.Done;
    public float maxSpeed; // current maximum speed owner is allowed to reach
    public float maxWantedSpeed; // maximum speed (temporarily) set by a CommandA
    //CGroudMoveType
    PathController pathController;
    Vector3 currWayPoint;
    Vector3 nextWayPoint;
    Vector3 waypointDir;
    Vector3 flatFrontDir;
    Vector3 lastAvoidanceDir;
    float turnRate = 0.1f;  /// maximum angular speed (angular units/frame)
    float turnSpeed = 0f;   /// current angular speed (angular units/frame)
    float turnAccel = 0f;   /// angular acceleration (angular units/frame^2)
    float accRate = 0.01f;
    float decRate = 0.01f;
    float wantedSpeed = 0.0f;
    float currentSpeed = 0.0f;
    float deltaSpeed = 0.0f;
    bool atGoal = false;
    bool atEndOfPath = false;
    bool wantRepath = false;
    float currWayPointDist = 0.0f;
    float prevWayPointDist = 0.0f;
    float goalRadius = 0.0f;                /// original radius passed to StartMoving*
    int pathID = 0;
    int nextObstacleAvoidanceFrame = 0;
    int wantedHeading = 0;
    bool pushResistant = true;

    bool reversing = false;
    bool idling = false;
    int numIdlingUpdates = 0;
    int numIdlingSlowUpdates = 0;
    public MoveType(Unit owner)
    {
        this.owner = owner;
        this.goalPos = owner.pos;
        this.oldPos = owner.pos;
        this.oldSlowUpdatePos = oldPos;
        this.progressState = ProgressState.Done;
        this.maxSpeed = owner.unitDef.speed / Game.GAME_SPEED;
        this.maxWantedSpeed = owner.unitDef.speed / Game.GAME_SPEED;
        this.pathController = new PathController(owner);
        this.currWayPoint = Vector3.zero;
        this.nextWayPoint = Vector3.zero;
        this.flatFrontDir = Vector3.forward;
        this.lastAvoidanceDir = Vector3.zero;
        this.wantedHeading = 0;
        pushResistant = owner.unitDef.pushResistant;
        turnRate = Mathf.Max(owner.unitDef.turnRate, 1.0f);
        turnAccel = turnRate * 0.333f;
        accRate = Mathf.Max(0.01f, owner.unitDef.maxAcc);
        decRate = Mathf.Max(0.01f, owner.unitDef.maxDec);
    }
    public void StartMoving(Vector3 moveGoalPos, float moveGoalRadius)
    {
        goalPos = new Vector3(moveGoalPos.x, 0, moveGoalPos.z);
        goalRadius = moveGoalRadius;
        atGoal = PathMathUtils.SqrDistance2D(owner.pos, moveGoalPos) < PathMathUtils.Square(moveGoalRadius);
        atEndOfPath = false;
        progressState = ProgressState.Active;
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
    public void StartEngine()
    {
        if (pathID == 0)
        {
            pathID = GetNewPath();
        }
        nextObstacleAvoidanceFrame = Game.frameNum;
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
            owner.SetVelocityAndSpeed(Vector3.zero);
            currentSpeed = 0f;
        }
        wantedSpeed = 0f;
    }
    public void Failed()
    {
        StopEngine();
        progressState = ProgressState.Failed;
        //
    }
    public int GetNewPath()
    {
        if (PathMathUtils.SqrDistance2D(owner.pos, goalPos) <= PathMathUtils.Square(goalRadius))
        {
            return 0;
        }
        int newPathID = PathManager.Instance().RequestPath(owner, owner.moveDef, owner.pos, goalPos, goalRadius, true);
        if (newPathID != 0)
        {
            atGoal = false;
            atEndOfPath = false;
            currWayPoint = PathManager.Instance().NextWayPoint(owner, newPathID, 0, owner.pos, Mathf.Max(WAYPOINT_RADIUS, currentSpeed * 1.05f), true);
            nextWayPoint = PathManager.Instance().NextWayPoint(owner, newPathID, 0, currWayPoint, Mathf.Max(WAYPOINT_RADIUS, currentSpeed * 1.05f), true);

            pathController.SetRealGoalPosition(newPathID, goalPos);
            pathController.SetTempGoalPosition(newPathID, currWayPoint);
        }
        else
        {
            Failed();
        }
        return newPathID;
    }
    public bool Update()
    {
        int heading = owner.heading;
        UpdateOwnerAccelAndHeading();
        UpdateOwnerPos(owner.speed, owner.frontdir * (owner.speedw + deltaSpeed));
        HandleObjectCollisions();
        return OwnerMoved(heading, owner.pos - oldPos, new Vector3(1e-4f, 1e-6f, 1e-4f));
    }
    public void SlowUpdate()
    {
        if (progressState == ProgressState.Active)
        {
            if (pathID != 0)
            {
                if (idling)
                {
                    numIdlingSlowUpdates = Mathf.Min(16, numIdlingSlowUpdates + 1);
                }
                else
                {
                    numIdlingSlowUpdates = Mathf.Max(0, numIdlingSlowUpdates - 1);
                }
                if (numIdlingUpdates > Game.CIRCLE_DIVS / 2 / turnRate)
                {
                    if (numIdlingSlowUpdates < 16)
                    {
                        ReRequestPath(true);
                    }
                    else
                    {
                        Failed();
                    }
                }
            }
            else
            {
                ReRequestPath(true);
            }
            if (wantRepath)
            {
                ReRequestPath(true);
            }
        }
        if (owner.pos != oldSlowUpdatePos)
        {
            oldSlowUpdatePos = owner.pos;
            int newMapSquare = Game.GetSquare(owner.pos);
            if (newMapSquare != owner.mapSquare)
            {
                owner.mapSquare = newMapSquare;
                owner.Block();
            }
            QuadField.Instance().MovedUnit(owner);
        }
    }
    public void UpdateOwnerAccelAndHeading()
    {
        FollowPath();
    }
    public void FollowPath()
    {
        if (WantToStop())
        {
            currWayPoint.y = -1.0f;
            nextWayPoint.y = -1.0f;
            ChangeHeading(owner.heading);
            ChangeSpeed(0.0f);
        }
        else
        {
            prevWayPointDist = currWayPointDist;
            currWayPointDist = PathMathUtils.Distance2D(currWayPoint, owner.pos);
            float curGoalDistSq = PathMathUtils.SqrDistance2D(owner.pos, goalPos);
            float minGoalDistSq = PathMathUtils.Square(goalRadius);
            atGoal |= (curGoalDistSq <= minGoalDistSq);
            if (!atGoal)
            {
                if (!idling)
                {
                    numIdlingUpdates = Mathf.Max(0, numIdlingUpdates - 1);
                }
                else
                {
                    numIdlingUpdates = Mathf.Min(32768, numIdlingUpdates + 1);
                }
            }
            if (!atEndOfPath)
            {
                GetNextWayPoint();
            }
            else if (atGoal)
            {
                Arrived();
            }
            if (currWayPoint != owner.pos)
            {
                waypointDir = new Vector3(currWayPoint.x - owner.pos.x, 0, currWayPoint.z - owner.pos.z).normalized;
            }
            Vector3 rawWantedDir = waypointDir;
            Vector3 modWantedDir = GetObstacleAvoidanceDir(rawWantedDir);
            ChangeHeading(PathMathUtils.GetHeadingFromVector(modWantedDir.x, modWantedDir.z));
            ChangeSpeed(maxWantedSpeed);
        }
    }
    float BrakingDistance(float speed, float rate)
    {
        float time = speed / Mathf.Max(rate, 0.001f);
        float dist = 0.5f * rate * time * time;
        return dist;
    }
    public void ChangeHeading(int newHeading)
    {
        wantedHeading = newHeading;
        int rawDeltaHeading = pathController.GetDeltaHeading(pathID, newHeading, owner.heading, turnRate, turnAccel, BrakingDistance(turnSpeed, turnAccel), ref turnSpeed);
        owner.AddHeading(rawDeltaHeading);
        flatFrontDir = new Vector3(owner.frontdir.x, 0, owner.frontdir.z).normalized;
    }
    public void UpdateOwnerPos(Vector3 oldSpeedVector, Vector3 newSpeedVector)
    {
    }
    public void HandleObjectCollisions()
    {
    }
    public bool OwnerMoved(int oldHeading, Vector3 posDif, Vector3 cmpEps)
    {
        return false;
    }
    public void ChangeSpeed(float newWantedSpeed)
    {
        wantedSpeed = newWantedSpeed;
        if (wantedSpeed <= 0.0f && currentSpeed < 0.01f)
        {
            currentSpeed = 0.0f;
            deltaSpeed = 0.0f;
            return;
        }
        float targetSpeed = maxSpeed;
        if (currWayPoint.y == -1.0f && nextWayPoint.y == -1.0f)
        {
            targetSpeed = 0.0f;
        }
        else
        {
            if (wantedSpeed > 0.0f)
            {
                float curGoalDistSq = PathMathUtils.SqrDistance2D(owner.pos, goalPos);
                float minGoalDistSq = PathMathUtils.Square(BrakingDistance(currentSpeed, decRate));
                Vector3 waypointDif = !reversing ? waypointDir : -waypointDir;
                int turnDeltaHeading = owner.heading - PathMathUtils.GetHeadingFromVector(waypointDif.x, waypointDif.z);
                bool startBraking = curGoalDistSq <= minGoalDistSq;
                if (turnDeltaHeading != 0)
                {
                    float reqTurnAngle = Mathf.Abs(180.0f * (owner.heading - wantedHeading) / (Game.CIRCLE_DIVS / 2));
                    float maxTurnAngle = turnRate / Game.CIRCLE_DIVS * 360.0f;
                    float turnMaxSpeed = !reversing ? maxSpeed : 0f;
                    float turnModSpeed = turnMaxSpeed;
                    if (reqTurnAngle != 0.0f)
                        turnModSpeed *= Mathf.Clamp(maxTurnAngle / reqTurnAngle, 0.1f, 1.0f);
                    if (waypointDir.sqrMagnitude > 0.1f)
                    {
                        if (!owner.unitDef.turnInPlace)
                        {
                            targetSpeed = Mathf.Clamp(turnModSpeed, Mathf.Min(owner.unitDef.turnInPlaceSpeedLimit, turnMaxSpeed), turnMaxSpeed);
                        }
                        else if (reqTurnAngle > owner.unitDef.turnInPlaceAngleLimit)
                        {
                            targetSpeed = turnModSpeed;
                        }
                    }
                    if (atEndOfPath)
                    {
                        targetSpeed = Mathf.Min(targetSpeed, (currWayPointDist * Mathf.PI) / (Game.CIRCLE_DIVS / turnRate));
                    }
                }
                targetSpeed *= (1 - (startBraking ? 1 : 0));
                targetSpeed *= (1 - (WantToStop() ? 1 : 0));
                targetSpeed = Mathf.Min(targetSpeed, wantedSpeed);
            }
            else
            {
                targetSpeed = 0.0f;
            }
        }
        deltaSpeed = pathController.GetDeltaSpeed(
            pathID,
            targetSpeed,
            currentSpeed,
            accRate,
            decRate,
            false,
            reversing
        );
    }
    public bool WantToStop()
    {
        return pathID == 0 && atEndOfPath;
    }
    public bool CanGetNextWayPoint()
    {
        if (pathID == 0)
        {
            return false;
        }
        if (currWayPoint.y != -1.0f && nextWayPoint.y != -1.0f)
        {
            Vector3 pos = owner.pos;
            Vector3 cwp = currWayPoint;
            Vector3 nwp = nextWayPoint;
            int dirSign = !reversing ? 1 : -1;
            float turnFrames = Game.CIRCLE_DIVS / turnRate;
            float turnRadius = (owner.speedw * turnFrames) / Mathf.PI;
            float waypointDot = Mathf.Clamp(Vector3.Dot(waypointDir, flatFrontDir * dirSign), -1.0f, 1.0f);
            if (currWayPointDist > turnRadius * 2.0f)
            {
                return false;
            }
            if (currWayPointDist > Game.SQUARE_SIZE && waypointDot > 0.995f)
            {
                return false;
            }
            int xmin = Mathf.Min((int)cwp.x / Game.SQUARE_SIZE, (int)pos.x / Game.SQUARE_SIZE);
            int xmax = Mathf.Max((int)cwp.x / Game.SQUARE_SIZE, (int)pos.x / Game.SQUARE_SIZE);
            int zmin = Mathf.Min((int)cwp.z / Game.SQUARE_SIZE, (int)pos.z / Game.SQUARE_SIZE);
            int zmax = Mathf.Max((int)cwp.z / Game.SQUARE_SIZE, (int)pos.z / Game.SQUARE_SIZE);
            MoveDef ownerMD = owner.moveDef;
            for (int x = xmin; x < xmax; x++)
            {
                for (int z = zmin; z < zmax; z++)
                {
                    if (ownerMD.TestMoveSquare(owner, x, z, owner.speed, true, true, true))
                    {
                        continue;
                    }
                    if ((pos - cwp).sqrMagnitude > PathMathUtils.Square(Game.SQUARE_SIZE)
                        && Vector3.Dot(pos - cwp, flatFrontDir) >= 0.0f)
                    {
                        return false;
                    }
                }
            }
            float curGoalDistSq = PathMathUtils.SqrDistance2D(currWayPoint, goalPos);
            float minGoalDistSq = PathMathUtils.Square(goalRadius);
            atEndOfPath |= (curGoalDistSq <= minGoalDistSq);
            if (atEndOfPath)
            {
                currWayPoint = goalPos;
                nextWayPoint = goalPos;
                return false;
            }
        }
        return true;
    }
    public void GetNextWayPoint()
    {
        if (CanGetNextWayPoint())
        {
            currWayPoint = nextWayPoint;
            nextWayPoint = PathManager.Instance().NextWayPoint(owner, pathID, 0, currWayPoint, 1.25f * Game.SQUARE_SIZE, true);
        }
        if (nextWayPoint.x == -1.0f && nextWayPoint.z == -1.0f)
        {
            Failed();
        }
        else
        {
            //var CWP_BLOCK_MASK = PathMathUtils.SquareIsBlocked(owner.moveDef, currWayPoint, owner);
            //var NWP_BLOCK_MASK = PathMathUtils.SquareIsBlocked(owner.moveDef, nextWayPoint, owner);
            //if ((CWP_BLOCK_MASK & CMoveMath::BLOCK_STRUCTURE) != 0 || (NWP_BLOCK_MASK & CMoveMath::BLOCK_STRUCTURE) != 0)
            //{
            //    // this can happen if we crushed a non-blocking feature
            //    // and it spawned another feature which we cannot crush
            //    // (eg.) --> repath
            //    ReRequestPath(false);
            //}
        }
    }
    public void Arrived()
    {
        if (progressState == ProgressState.Active)
        {
            StopEngine();
            progressState = ProgressState.Done;
        }
    }
    public Vector3 GetObstacleAvoidanceDir(Vector3 desiredDir)
    {
        return Vector3.zero;
    }
}