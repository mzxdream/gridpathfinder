using UnityEngine;

public class MoveType
{
    public const int GAME_SPEED = 30;
    public const float SQUARE_SIZE = 1f;
    public const float WAYPOINT_RADIUS = 1.25f * SQUARE_SIZE;

    public Unit owner;
    public Vector3 goalPos;
    public Vector3 oldPos;
    public Vector3 oldSlowUpdatePos;
    public enum ProgressState { Done, Active, Failed };
    public ProgressState progressState = ProgressState.Done;
    private float maxSpeed;
    private float maxWantedSpeed;
    //private float maneuverLeash;
    private bool useHeading = true;

    PathController pathController;
    Vector3 currWayPoint;
    Vector3 nextWayPoint;
    Vector3 waypointDir;
    Vector3 flatFrontDir;
    Vector3 lastAvoidanceDir;
    Vector3 mainHeadingPos;
    //Vector3 skidRotVector;
    float turnRate = 0.1f;
    float turnSpeed = 0f;
    float turnAccel = 0f;
    float accRate = 0.01f;
    float decRate = 0.01f;
    //float myGravity = 0.0f;

    float maxReverseDist = 0.0f;
    float minReverseAngle = 0.0f;
    float maxReverseSpeed = 0.0f;
    float sqSkidSpeedMult = 0.95f;

    float wantedSpeed = 0.0f;
    float currentSpeed = 0.0f;
    float deltaSpeed = 0.0f;

    float currWayPointDist = 0.0f;
    float prevWayPointDist = 0.0f;

    float goalRadius = 0.0f;                /// original radius passed to StartMoving*
	float ownerRadius = 0.0f;               /// owner MoveDef footprint radius
	float extraRadius = 0.0f;               /// max(0, ownerRadius - goalRadius) if goal-pos is valid, 0 otherwise

    //float skidRotSpeed = 0.0f;              /// rotational speed when skidding (radians / (GAME_SPEED frames))
    //float skidRotAccel = 0.0f;              /// rotational acceleration when skidding (radians / (GAME_SPEED frames^2))

    int pathID = 0;
    //int nextObstacleAvoidanceFrame = 0;

    int numIdlingUpdates = 0;      /// {in, de}creased every Update if idling is true/false and pathId != 0
	int numIdlingSlowUpdates = 0;  /// {in, de}creased every SlowUpdate if idling is true/false and pathId != 0

    short wantedHeading = 0;
    short minScriptChangeHeading = 0;       /// minimum required turn-angle before script->ChangeHeading is called

    bool atGoal = false;
    bool atEndOfPath = false;
    bool wantRepath = false;

    bool reversing = false;
    bool idling = false;
    bool pushResistant = false;
    bool canReverse = false;
    bool useMainHeading = false;            /// if true, turn toward mainHeadingPos until weapons[0] can TryTarget() it
    public MoveType(Unit owner)
    {
        this.owner = owner;
        goalPos = owner.pos;
        oldPos = owner.pos;
        oldSlowUpdatePos = oldPos;
        maxSpeed = owner.moveDef.speed / GAME_SPEED;
        maxWantedSpeed = owner.moveDef.speed / GAME_SPEED;
        pathController = new PathController(owner);
        currWayPoint = Vector3.zero;
        nextWayPoint = Vector3.zero;
        flatFrontDir = Vector3.right;
        lastAvoidanceDir = Vector3.zero;
        mainHeadingPos = Vector3.zero;
        wantedHeading = 0;
        pushResistant = owner.moveDef.pushResistant;
        canReverse = owner.moveDef.rSpeed > 0f;
        maxReverseSpeed = owner.moveDef.rSpeed / GAME_SPEED;
        turnRate = Mathf.Clamp(owner.moveDef.turnRate, 1.0f, 32768.0f);
        turnAccel = turnRate * 0.333f;
        ownerRadius = owner.moveDef.radius;
    }
    public void StartMoving(Vector3 moveGoalPos, float moveGoalRadius)
    {
        float deltaRaidus = Mathf.Max(0f, ownerRadius - moveGoalRadius);
        goalPos = new Vector3(moveGoalPos.x, 0, moveGoalPos.z);
        goalRadius = moveGoalRadius;
        extraRadius = owner.moveDef.TestMoveSquare(null, moveGoalPos, Vector3.zero, true, true) ? 0 : deltaRaidus;

        atGoal = PathMathUtils.SqrDistance2D(owner.pos, moveGoalPos) < (goalRadius + extraRadius) * (goalRadius + extraRadius);
        atEndOfPath = false;

        useMainHeading = false;
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
        if (pathID != 0)
        {
            PathManager.Instance().UpdatePath(owner, pathID);
        }
        //nextObstacleAvoidanceFrame = 0;
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
        int newPathID = 0;
        if (PathMathUtils.SqrDistance2D(owner.pos, goalPos) <= (goalRadius + extraRadius) * (goalRadius + extraRadius))
        {
            return newPathID;
        }
        newPathID = PathManager.Instance().RequestPath(owner, owner.moveDef, owner.pos, goalPos, goalRadius + extraRadius, true);
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
        short heading = owner.heading;

        // these must be executed even when stunned (so
        // units do not get buried by restoring terrain)
        UpdateOwnerAccelAndHeading();
        UpdateOwnerPos(owner->speed, calcSpeedVectorFuncs[modInfo.allowGroundUnitGravity](owner, this, deltaSpeed, myGravity));
        HandleObjectCollisions();
        AdjustPosToWaterLine();
        return (OwnerMoved(heading, owner->pos - oldPos, float3(float3::cmp_eps(), float3::cmp_eps() * 1e-2f, float3::cmp_eps())));
    }
    public void UpdateOwnerAccelAndHeading()
    {
        if (owner.stunned)
        {
            ChangeSpeed(0, false);
            return;
        }
        FollowPath();
    }
    public void ChangeSpeed(float newWantedSpeed, bool wantReverse, bool fpsMode = false)
    {
        // round low speeds to zero
        if ((wantedSpeed = newWantedSpeed) <= 0.0f && currentSpeed < 0.01f)
        {
            currentSpeed = 0.0f;
            deltaSpeed = 0.0f;
            return;
        }
        // first calculate the "unrestricted" speed and acceleration
        float targetSpeed = PathMathUtils.Mix(maxSpeed, maxReverseSpeed, wantReverse ? 1 : 0);
        // don't move until we have an actual path, trying to hide queuing
        // lag is too dangerous since units can blindly drive into objects,
        // cliffs, etc. (requires the QTPFS idle-check in Update)
        if (currWayPoint.y == -1.0f && nextWayPoint.y == -1.0f)
        {
            targetSpeed = 0.0f;
        }
        else
        {
            if (wantedSpeed > 0.0f)
            {
                targetSpeed = wantedSpeed;
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
            wantReverse,
            reversing
        );
    }
    public bool WantToStop()
    {
        return pathID == 0 && atEndOfPath;
    }
    public bool FollowPath()
    {
        bool wantReverse = false;

        if (WantToStop())
        {
            currWayPoint.y = -1.0f;
            nextWayPoint.y = -1.0f;

            minScriptChangeHeading();
            ChangeSpeed(0.0f, false);
        }
        else
        {
            ASSERT_SYNCED(currWayPoint);
            ASSERT_SYNCED(nextWayPoint);
            ASSERT_SYNCED(owner->pos);

            const float3&opos = owner->pos;
            const float3&ovel = owner->speed;
            const float3&ffd = flatFrontDir;
            const float3&cwp = currWayPoint;

            prevWayPointDist = currWayPointDist;
            currWayPointDist = currWayPoint.distance2D(opos);

            {
                // NOTE:
                //   uses owner->pos instead of currWayPoint (ie. not the same as atEndOfPath)
                //
                //   if our first command is a build-order, then goal-radius is set to our build-range
                //   and we cannot increase tolerance safely (otherwise the unit might stop when still
                //   outside its range and fail to start construction)
                //
                //   units moving faster than <minGoalDist> elmos per frame might overshoot their goal
                //   the last two atGoal conditions will just cause flatFrontDir to be selected as the
                //   "wanted" direction when this happens
                const float curGoalDistSq = (opos - goalPos).SqLength2D();
                const float minGoalDistSq = (UNIT_HAS_MOVE_CMD(owner)) ?
                    Square((goalRadius + extraRadius) * (numIdlingSlowUpdates + 1)) :
                    Square((goalRadius + extraRadius));
                const float spdGoalDistSq = Square(currentSpeed * 1.05f);

                atGoal |= (curGoalDistSq <= minGoalDistSq);
                atGoal |= ((curGoalDistSq <= spdGoalDistSq) && !reversing && (ffd.dot(goalPos - opos) > 0.0f && ffd.dot(goalPos - (opos + ovel)) <= 0.0f));
                atGoal |= ((curGoalDistSq <= spdGoalDistSq) && reversing && (ffd.dot(goalPos - opos) < 0.0f && ffd.dot(goalPos - (opos + ovel)) >= 0.0f));
            }

            if (!atGoal)
            {
                numIdlingUpdates -= ((numIdlingUpdates > 0) * (1 - idling));
                numIdlingUpdates += ((numIdlingUpdates < SPRING_MAX_HEADING) * idling);
            }

            // atEndOfPath never becomes true when useRawMovement, except via StopMoving
            if (!atEndOfPath && !useRawMovement)
            {
                SetNextWayPoint();
            }
            else
            {
                if (atGoal)
                    Arrived(false);
                else
                    ReRequestPath(false);
            }


            // set direction to waypoint AFTER requesting it; should not be a null-vector
            // do not compare y-components since these usually differ and only x&z matter
            float3 waypointVec;
            // float3 wpProjDists;

            if (!epscmp(cwp.x, opos.x, float3::cmp_eps()) || !epscmp(cwp.z, opos.z, float3::cmp_eps()))
            {
                waypointVec = (cwp - opos) * XZVector;
                waypointDir = waypointVec / waypointVec.Length();
                // wpProjDists = {math::fabs(waypointVec.dot(ffd)), 1.0f, math::fabs(waypointDir.dot(ffd))};
            }

            ASSERT_SYNCED(waypointVec);
            ASSERT_SYNCED(waypointDir);

            wantReverse = WantReverse(waypointDir, ffd);

            // apply obstacle avoidance (steering), prevent unit from chasing its own tail if already at goal
            const float3 rawWantedDir = waypointDir * Sign(int(!wantReverse));
            const float3&modWantedDir = GetObstacleAvoidanceDir(mix(ffd, rawWantedDir, !atGoal));
            // const float3& modWantedDir = GetObstacleAvoidanceDir(mix(ffd, rawWantedDir, (!atGoal) && (wpProjDists.x > wpProjDists.y || wpProjDists.z < 0.995f)));

            ChangeHeading(GetHeadingFromVector(modWantedDir.x, modWantedDir.z));
            ChangeSpeed(maxWantedSpeed, wantReverse);
        }

        pathManager->UpdatePath(owner, pathID);
        return wantReverse;
    }
}