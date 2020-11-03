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
    public bool CanSetNextWayPoint()
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
            if (PathManager.Instance().PathUpdated(pathID))
            {
                cwp = PathManager.Instance().NextWayPoint(owner, pathID, 0, pos, Mathf.Max(WAYPOINT_RADIUS, currentSpeed * 1.05f), true);
                nwp = PathManager.Instance().NextWayPoint(owner, pathID, 0, cwp, Mathf.Max(WAYPOINT_RADIUS, currentSpeed * 1.05f), true);
            }
            int dirSign = (int)PathMathUtils.Sign(!reversing ? 1f : 0f);
            float absTurnSpeed = Mathf.Max(0.0001f, Mathf.Abs(turnSpeed));
            float framesToTurn = 32768 * 2 / absTurnSpeed;
            float turnRadius = Mathf.Max((currentSpeed * framesToTurn) / (2 * Mathf.PI), currentSpeed * 1.05f);
            float waypointDot = Mathf.Clamp(Vector3.Dot(waypointDir, (flatFrontDir * dirSign)), -1.0f, 1.0f);
            // wp outside turning circle
            if (currWayPointDist > (turnRadius * 2.0f))
                return false;
            // wp inside but ~straight ahead and not reached within one tick
            if (currWayPointDist > Mathf.Max(Game.SQUARE_SIZE * 1.0f, currentSpeed * 1.05f) && waypointDot >= 0.995f)
                return false;
            {
                // check the rectangle between pos and cwp for obstacles
                // if still further than SS elmos from waypoint, disallow skipping
                // note: can somehow cause units to move in circles near obstacles
                // (mantis3718) if rectangle is too generous in size
                float minSpeedMod = 0;
                int maxBlockBit = 0;
                bool rangeTest = owner.moveDef.TestMoveSquareRange(owner, Vector3.Min(cwp, pos), Vector3.Max(cwp, pos), owner.speed, true, true, true, ref minSpeedMod, ref maxBlockBit);
                bool allowSkip = ((cwp - pos).sqrMagnitude <= PathMathUtils.Square(Game.SQUARE_SIZE));
                // CanSetNextWayPoint may return true if (allowSkip || rangeTest)
                if (!allowSkip && !rangeTest)
                    return false;
            }
            {
                float curGoalDistSq = PathMathUtils.SqrDistance2D(currWayPoint, goalPos);
                float minGoalDistSq = PathMathUtils.Square((goalRadius + extraRadius));

                // trigger Arrived on the next Update (only if we have non-temporary waypoints)
                // note:
                //   coldet can (very rarely) interfere with this, causing it to remain false
                //   a unit would then keep moving along its final waypoint-direction forever
                //   if atGoal, so we require waypointDir to always be updated in FollowPath
                //   (checking curr == next is not perfect, becomes true a waypoint too early)
                //
                // atEndOfPath |= (currWayPoint == nextWayPoint);
                atEndOfPath |= (curGoalDistSq <= minGoalDistSq);
            }

            if (atEndOfPath)
            {
                currWayPoint = goalPos;
                nextWayPoint = goalPos;
                return false;
            }
        }
        return true;
    }
    public void SetNextWayPoint()
    {
        if (CanSetNextWayPoint())
        {
            pathController.SetTempGoalPosition(pathID, nextWayPoint);
            currWayPoint = nextWayPoint;
            nextWayPoint = PathManager.Instance().NextWayPoint(owner, pathID, 0, currWayPoint, Mathf.Max(WAYPOINT_RADIUS, currentSpeed * 1.05f), true);
        }
        if (nextWayPoint.x == -1.0f && nextWayPoint.z == -1.0f)
        {
            Failed();
            return;
        }
        var CWP_BLOCK_MASK = PathMathUtils.SquareIsBlocked(owner.moveDef, currWayPoint, owner);
        var NWP_BLOCK_MASK = PathMathUtils.SquareIsBlocked(owner.moveDef, nextWayPoint, owner);
        if ((CWP_BLOCK_MASK & (int)PathMathUtils.BlockTypes.BLOCK_STRUCTURE) == 0 && (NWP_BLOCK_MASK & (int)PathMathUtils.BlockTypes.BLOCK_STRUCTURE) == 0)
            return;
        ReRequestPath(false);
    }
    public void Arrived()
    {
        if (progressState == ProgressState.Active)
        {
            StopEngine();
            progressState = ProgressState.Done;
            //TODO
        }
    }
       public void GetNextWayPoint()
    {
    }
    public Vector3 GetObstacleAvoidanceDir(Vector3 desiredDir)
    {
        if (WantToStop())
            return flatFrontDir;

        // Speed-optimizer. Reduces the times this system is run.
        if (Game.frameNum < nextObstacleAvoidanceFrame)
            return lastAvoidanceDir;
        Vector3 avoidanceVec = Vector3.zero;
        var avoidanceDir = desiredDir;
        lastAvoidanceDir = desiredDir;
        nextObstacleAvoidanceFrame = Game.frameNum + 1;
        Unit avoider = owner;
        MoveDef avoiderMD = avoider.moveDef;

        // degenerate case: if facing anti-parallel to desired direction,
        // do not actively avoid obstacles since that can interfere with
        // normal waypoint steering (if the final avoidanceDir demands a
        // turn in the opposite direction of desiredDir)
        if (Vector3.Dot(avoider.frontdir, desiredDir) < 0.0f)
            return lastAvoidanceDir;
        float AVOIDER_DIR_WEIGHT = 1.0f;
        float DESIRED_DIR_WEIGHT = 0.5f;
        float LAST_DIR_MIX_ALPHA = 0.7f;
        float MAX_AVOIDEE_COSINE = Mathf.Cos(120.0f * Mathf.PI / 180f);
        // now we do the obstacle avoidance proper
        // avoider always uses its never-rotated MoveDef footprint
        // note: should increase radius for smaller turnAccel values
        float avoidanceRadius = Mathf.Max(currentSpeed, 1.0f) * (avoider.radius * 2.0f);
        float avoiderRadius = avoiderMD.CalcFootPrintMinExteriorRadius();

        QuadFieldQuery qfQuery = new QuadFieldQuery();
        QuadField.Instance().GetSolidsExact(qfQuery, avoider.pos, avoidanceRadius);

        foreach (var avoidee in qfQuery.units)
        {
            MoveDef avoideeMD = avoidee.moveDef;
            if (avoidee == owner)
                continue;
            bool avoideeMobile = true;
            bool avoideeMovable = !avoideeMD.pushResistant;
            Vector3 avoideeVector = (avoider.pos + avoider.speed) - (avoidee.pos + avoidee.speed);
            // use the avoidee's MoveDef footprint as radius if it is mobile
            // use the avoidee's Unit (not UnitDef) footprint as radius otherwise
            float avoideeRadius = avoideeMD.CalcFootPrintMinExteriorRadius();
            float avoidanceRadiusSum = avoiderRadius + avoideeRadius;
            float avoidanceMassSum = avoider.mass + avoidee.mass;
            float avoideeMassScale = avoidee.mass / avoidanceMassSum;
            float avoideeDistSq = avoideeVector.sqrMagnitude;
            float avoideeDist = Mathf.Sqrt(avoideeDistSq) + 0.01f;
            // do not bother steering around idling MOBILE objects
            // (since collision handling will just push them aside)
            if (avoideeMovable)
            {
                if (!avoiderMD.avoidMobilesOnPath || (!avoidee.moving && avoidee.allyteam == avoider.allyteam))
                    continue;
            }
            // ignore objects that are more than this many degrees off-center from us
            // NOTE:
            //   if MAX_AVOIDEE_COSINE is too small, then this condition can be true
            //   one frame and false the next (after avoider has turned) causing the
            //   avoidance vector to oscillate --> units with turnInPlace = true will
            //   slow to a crawl as a result
            if (Vector3.Dot(avoider.frontdir, -(avoideeVector / avoideeDist)) < MAX_AVOIDEE_COSINE)
                continue;
            if (avoideeDistSq >= PathMathUtils.Square(Mathf.Max(currentSpeed, 1.0f) * Game.GAME_SPEED + avoidanceRadiusSum))
                continue;
            if (avoideeDistSq >= PathMathUtils.SqrDistance2D(avoider.pos, goalPos))
                continue;
            float avoiderTurnSign = -PathMathUtils.Sign(Vector3.Dot(avoidee.pos, avoider.rightdir) - Vector3.Dot(avoider.pos, avoider.rightdir));
            float avoideeTurnSign = -PathMathUtils.Sign(Vector3.Dot(avoider.pos, avoidee.rightdir) - Vector3.Dot(avoidee.pos, avoidee.rightdir));

            // for mobile units, avoidance-response is modulated by angle
            // between avoidee's and avoider's frontdir such that maximal
            // avoidance occurs when they are anti-parallel
            float avoidanceCosAngle = Mathf.Clamp(Vector3.Dot(avoider.frontdir, avoidee.frontdir), -1.0f, 1.0f);
            float avoidanceResponse = (1.0f - avoidanceCosAngle) + 0.1f;
            float avoidanceFallOff = (1.0f - Mathf.Min(1.0f, avoideeDist / (5.0f * avoidanceRadiusSum)));
            if (avoidanceCosAngle < 0.0f)
                avoiderTurnSign = Mathf.Max(avoiderTurnSign, avoideeTurnSign);
            avoidanceDir = avoider.rightdir * AVOIDER_DIR_WEIGHT * avoiderTurnSign;
            avoidanceVec += (avoidanceDir * avoidanceResponse * avoidanceFallOff * avoideeMassScale);
        }
        avoidanceDir = (PathMathUtils.MixVec3(desiredDir, avoidanceVec, DESIRED_DIR_WEIGHT)).normalized;
        avoidanceDir = (PathMathUtils.MixVec3(avoidanceDir, lastAvoidanceDir, LAST_DIR_MIX_ALPHA)).normalized;
        return (lastAvoidanceDir = avoidanceDir);
    }

}