using UnityEngine;

public class MoveType
{
    public const float WAYPOINT_RADIUS = 1.25f * Game.SQUARE_SIZE;
    //AMoveType begin
    public Unit owner;
    public Vector3 goalPos;
    public Vector3 oldPos; // owner position at last Update()
    public Vector3 oldSlowUpdatePos; // owner position at last SlowUpdate()
    public enum ProgressState { Done, Active, Failed };
    public ProgressState progressState = ProgressState.Done;
    private float maxSpeed; // current maximum speed owner is allowed to reach
    private float maxWantedSpeed; // maximum speed (temporarily) set by a CommandA
    //private float maneuverLeash;
    //private bool useHeading = true;

    //CGroudMoveType
    PathController pathController;
    Vector3 currWayPoint;
    Vector3 nextWayPoint;
    Vector3 waypointDir;
    Vector3 flatFrontDir;
    Vector3 lastAvoidanceDir;
    //Vector3 mainHeadingPos;
    //Vector3 skidRotVector;  /// vector orthogonal to skidDi
    float turnRate = 0.1f;  /// maximum angular speed (angular units/frame)
    float turnSpeed = 0f;   /// current angular speed (angular units/frame)
    float turnAccel = 0f;   /// angular acceleration (angular units/frame^2)
    float accRate = 0.01f;
    float decRate = 0.01f;
    //float myGravity = 0.0f;

    float maxReverseDist = 0.0f;
    float minReverseAngle = 0.0f;
    float maxReverseSpeed = 0.0f;
    //float sqSkidSpeedMult = 0.95f;

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
    int nextObstacleAvoidanceFrame = 0;

    int numIdlingUpdates = 0;      /// {in, de}creased every Update if idling is true/false and pathId != 0
	int numIdlingSlowUpdates = 0;  /// {in, de}creased every SlowUpdate if idling is true/false and pathId != 0

    int wantedHeading = 0;
    int minScriptChangeHeading = 0;       /// minimum required turn-angle before script->ChangeHeading is called

    bool atGoal = false;
    bool atEndOfPath = false;
    bool wantRepath = false;

    bool reversing = false;
    bool idling = false;
    bool pushResistant = false;
    bool canReverse = false;
    //bool useMainHeading = false;            /// if true, turn toward mainHeadingPos until weapons[0] can TryTarget() it
    public MoveType(Unit owner)
    {
        this.owner = owner;
        goalPos = owner.pos;
        oldPos = owner.pos;
        oldSlowUpdatePos = oldPos;
        maxSpeed = owner.moveDef.speed / Game.GAME_SPEED;
        maxWantedSpeed = owner.moveDef.speed / Game.GAME_SPEED;

        pathController = new PathController(owner);
        currWayPoint = Vector3.zero;
        nextWayPoint = Vector3.zero;
        flatFrontDir = Vector3.forward;
        lastAvoidanceDir = Vector3.zero;
        wantedHeading = 0;
        pushResistant = owner.moveDef.pushResistant;
        canReverse = owner.moveDef.rSpeed > 0f;
        maxReverseSpeed = owner.moveDef.rSpeed / Game.GAME_SPEED;
        turnRate = Mathf.Clamp(owner.moveDef.turnRate, 1.0f, 32767.0f);
        turnAccel = turnRate * 0.333f;
        accRate = Mathf.Max(0.01f, owner.moveDef.maxAcc);
        decRate = Mathf.Max(0.01f, owner.moveDef.maxDec);
        ownerRadius = owner.moveDef.CalcFootPrintMinExteriorRadius();
    }
    public void StartMoving(Vector3 moveGoalPos, float moveGoalRadius)
    {
        float deltaRaidus = Mathf.Max(0f, ownerRadius - moveGoalRadius);
        goalPos = new Vector3(moveGoalPos.x, 0, moveGoalPos.z);
        goalRadius = moveGoalRadius;
        extraRadius = owner.moveDef.TestMoveSquare(null, moveGoalPos, Vector3.zero, true, true) ? 0 : deltaRaidus;

        atGoal = PathMathUtils.SqrDistance2D(owner.pos, moveGoalPos) < PathMathUtils.Square(goalRadius + extraRadius);
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
        if (pathID != 0)
        {
            PathManager.Instance().UpdatePath(owner, pathID);
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
        int newPathID = 0;
        if (PathMathUtils.SqrDistance2D(owner.pos, goalPos) <= PathMathUtils.Square(goalRadius + extraRadius))
        {
            return newPathID;
        }
        newPathID = PathManager.Instance().RequestPath(owner, owner.moveDef, owner.pos, goalPos, goalRadius + extraRadius, true);
        if (newPathID != 0)
        {
            atGoal = false;
            atEndOfPath = false;
            currWayPoint = PathManager.Instance().NextWayPoint(owner, newPathID, 0,    owner.pos, Mathf.Max(WAYPOINT_RADIUS, currentSpeed * 1.05f), true);
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

        // these must be executed even when stunned (so
        // units do not get buried by restoring terrain)
        UpdateOwnerAccelAndHeading();
        //UpdateOwnerPos(owner->speed, calcSpeedVectorFuncs[modInfo.allowGroundUnitGravity](owner, this, deltaSpeed, myGravity));
        //HandleObjectCollisions();
        //AdjustPosToWaterLine();
        //return (OwnerMoved(heading, owner->pos - oldPos, float3(float3::cmp_eps(), float3::cmp_eps() * 1e-2f, float3::cmp_eps())));
        return true;
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
    // The distance the unit will move before stopping,
    // starting from given speed and applying maximum
    // brake rate.
    public float BrakingDistance(float speed, float rate)
    {
        float time = speed / Mathf.Max(rate, 0.001f);
        float dist = 0.5f * rate * time * time;
        return dist;
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
                float groundSpeedMod = 1.0f;
                float curGoalDistSq = PathMathUtils.SqrDistance2D(owner.pos, goalPos);
                float minGoalDistSq = PathMathUtils.Square(BrakingDistance(currentSpeed, decRate));
                Vector3 waypointDifFwd = waypointDir;
                Vector3 waypointDifRev = -waypointDifFwd;
                Vector3 waypointDif = PathMathUtils.MixVec3(waypointDifFwd, waypointDifRev, reversing ? 1 : 0);
                int turnDeltaHeading = owner.heading - PathMathUtils.GetHeadingFromVector(waypointDif.x, waypointDif.z);

                bool startBraking = (curGoalDistSq <= minGoalDistSq && !fpsMode);

                if (!fpsMode && turnDeltaHeading != 0) {
                    // only auto-adjust speed for turns when not in FPS mode
                    float reqTurnAngle = Mathf.Abs(180.0f * (owner.heading - wantedHeading) / 32768);
                    float maxTurnAngle = (turnRate / (32768 * 2)) * 360.0f;
                    float turnMaxSpeed = PathMathUtils.Mix(maxSpeed, maxReverseSpeed, reversing ? 1 : 0);
                    float turnModSpeed = turnMaxSpeed;
                    if (reqTurnAngle != 0.0f)
                        turnModSpeed *= Mathf.Clamp(maxTurnAngle / reqTurnAngle, 0.1f, 1.0f);
                    if (waypointDir.sqrMagnitude > 0.1f)
                    {
                        if (!owner.moveDef.turnInPlace)
                        {
                            // never let speed drop below TIPSL, but limit TIPSL itself to turnMaxSpeed
                            targetSpeed = Mathf.Clamp(turnModSpeed, Mathf.Min(owner.moveDef.turnInPlaceSpeedLimit, turnMaxSpeed), turnMaxSpeed);
                        }
                        else
                        {
                            targetSpeed = PathMathUtils.Mix(targetSpeed, turnModSpeed, (reqTurnAngle > owner.moveDef.turnInPlaceAngleLimit) ? 1.0f : 0.0f);
                        }
                    }
                    if (atEndOfPath)
                    {
                        // at this point, Update() will no longer call SetNextWayPoint()
                        // and we must slow down to prevent entering an infinite circle
                        // base ftt on maximum turning speed
                        float absTurnSpeed = turnRate;
                        float framesToTurn = 32768 * 2 / absTurnSpeed;
                        targetSpeed = Mathf.Min(targetSpeed, (currWayPointDist * Mathf.PI) / framesToTurn);
                    }
                }
                // now apply the terrain and command restrictions
                // NOTE:
                //   if wantedSpeed > targetSpeed, the unit will
                //   not accelerate to speed > targetSpeed unless
                //   its actual max{Reverse}Speed is also changed
                //
                //   raise wantedSpeed iff the terrain-modifier is
                //   larger than 1 (so units still get their speed
                //   bonus correctly), otherwise leave it untouched
                //
                //   disallow changing speed (except to zero) without
                //   a path if not in FPS mode (FIXME: legacy PFS can
                //   return path when none should exist, mantis3720)
                wantedSpeed *= Mathf.Max(groundSpeedMod, 1.0f);
                targetSpeed *= groundSpeedMod;
                targetSpeed *= (1 - (startBraking ? 1 : 0));
                targetSpeed *= ((1 - (WantToStop() ? 1 : 0)) > 0 || fpsMode) ? 1 : 0;
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
            wantReverse,
            reversing
        );
    }
    public bool WantToStop()
    {
        return pathID == 0 && atEndOfPath;
    }
    public void ChangeHeading(int newHeading)
    {
        // model rotational inertia (more realistic for ships)
        int rawDeltaHeading = pathController.GetDeltaHeading(pathID, (wantedHeading = newHeading), owner.heading, turnRate, turnAccel, BrakingDistance(turnSpeed, turnAccel), ref turnSpeed);
        int absDeltaHeading = rawDeltaHeading * (int)PathMathUtils.Sign(rawDeltaHeading);
        //if (absDeltaHeading >= minScriptChangeHeading)
        //    owner->script->ChangeHeading(rawDeltaHeading);
        owner.AddHeading(rawDeltaHeading, !owner.upright, false);
        flatFrontDir = new Vector3(owner.frontdir.x, 0, owner.frontdir.z).normalized;
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
    public bool WantReverse(Vector3 wpDir, Vector3 ffDir)
    {
        return false;
    }
    public bool FollowPath()
    {
        bool wantReverse = false;

        if (WantToStop())
        {
            currWayPoint.y = -1.0f;
            nextWayPoint.y = -1.0f;
            ChangeHeading(owner.heading);
            ChangeSpeed(0.0f, false);
        }
        else
        {
            Vector3 opos = owner.pos;
            Vector3 ovel = owner.speed;
            Vector3 ffd = flatFrontDir;
            Vector3 cwp = currWayPoint;
            prevWayPointDist = currWayPointDist;
            currWayPointDist = PathMathUtils.Distance2D(currWayPoint, opos);
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
                float curGoalDistSq = PathMathUtils.SqrDistance2D(opos, goalPos);
                float minGoalDistSq = PathMathUtils.Square((goalRadius + extraRadius));
                float spdGoalDistSq = PathMathUtils.Square(currentSpeed * 1.05f);
                atGoal |= (curGoalDistSq <= minGoalDistSq);
                atGoal |= ((curGoalDistSq <= spdGoalDistSq) && !reversing && (Vector3.Dot(ffd, goalPos - opos) > 0.0f && Vector3.Dot(ffd, goalPos - (opos + ovel)) <= 0.0f));
                atGoal |= ((curGoalDistSq <= spdGoalDistSq) && reversing && (Vector3.Dot(ffd, goalPos - opos) < 0.0f && Vector3.Dot(ffd, goalPos - (opos + ovel)) >= 0.0f));
            }
            if (!atGoal)
            {
                numIdlingUpdates -= ((numIdlingUpdates > 0 ? 1 : 0) * (1 - (idling ? 1 : 0)));
                numIdlingUpdates += ((numIdlingUpdates < 32768 ? 1 : 0) * (idling ? 1 : 0));
            }

            // atEndOfPath never becomes true when useRawMovement, except via StopMoving
            if (!atEndOfPath)
            {
                SetNextWayPoint();
            }
            else
            {
                if (atGoal)
                    Arrived();
                else
                    ReRequestPath(false);
            }
            // set direction to waypoint AFTER requesting it; should not be a null-vector
            // do not compare y-components since these usually differ and only x&z matter
            Vector3 waypointVec;
            // float3 wpProjDists;

            if (!PathMathUtils.Epscmp0001(cwp.x, opos.x) || !PathMathUtils.Epscmp0001(cwp.z, opos.z))
            {
                waypointVec = (cwp - opos);
                waypointVec.y = 0f;
                waypointDir = waypointVec / waypointVec.magnitude;
                // wpProjDists = {math::fabs(waypointVec.dot(ffd)), 1.0f, math::fabs(waypointDir.dot(ffd))};
            }
            wantReverse = WantReverse(waypointDir, ffd);

            // apply obstacle avoidance (steering), prevent unit from chasing its own tail if already at goal
            var rawWantedDir = waypointDir * PathMathUtils.Sign(!wantReverse ? 1 : 0);
            var modWantedDir = GetObstacleAvoidanceDir(PathMathUtils.MixVec3(ffd, rawWantedDir, !atGoal ? 1 : 0));
            // const float3& modWantedDir = GetObstacleAvoidanceDir(mix(ffd, rawWantedDir, (!atGoal) && (wpProjDists.x > wpProjDists.y || wpProjDists.z < 0.995f)));

            ChangeHeading(PathMathUtils.GetHeadingFromVector(modWantedDir.x, modWantedDir.z));
            ChangeSpeed(maxWantedSpeed, wantReverse);
        }
        PathManager.Instance().UpdatePath(owner, pathID);
        return wantReverse;
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
        const float avoidanceRadius = std::max(currentSpeed, 1.0f) * (avoider->radius * 2.0f);
        const float avoiderRadius = avoiderMD->CalcFootPrintMinExteriorRadius();

	QuadFieldQuery qfQuery;
	quadField.GetSolidsExact(qfQuery, avoider->pos, avoidanceRadius, 0xFFFFFFFF, CSolidObject::CSTATE_BIT_SOLIDOBJECTS);

	for (const CSolidObject* avoidee: *qfQuery.solids) {
		const MoveDef* avoideeMD = avoidee->moveDef;
		const UnitDef* avoideeUD = dynamic_cast<const UnitDef*>(avoidee->GetDef());

		// cases in which there is no need to avoid this obstacle
		if (avoidee == owner)
			continue;
		// do not avoid statics (it interferes too much with PFS)
		if (avoideeMD == nullptr)
			continue;
		// ignore aircraft (or flying ground units)
		if (avoidee->IsInAir() || avoidee->IsFlying())
			continue;
		if (CMoveMath::IsNonBlocking(*avoiderMD, avoidee, avoider))
			continue;
		if (!CMoveMath::CrushResistant(*avoiderMD, avoidee))
			continue;

		const bool avoideeMobile  = (avoideeMD != nullptr);
		const bool avoideeMovable = (avoideeUD != nullptr && !static_cast<const CUnit*>(avoidee)->moveType->IsPushResistant());

		const float3 avoideeVector = (avoider->pos + avoider->speed) - (avoidee->pos + avoidee->speed);

		// use the avoidee's MoveDef footprint as radius if it is mobile
		// use the avoidee's Unit (not UnitDef) footprint as radius otherwise
		const float avoideeRadius = avoideeMobile?
			avoideeMD->CalcFootPrintMinExteriorRadius():
			avoidee->CalcFootPrintMinExteriorRadius();
		const float avoidanceRadiusSum = avoiderRadius + avoideeRadius;
		const float avoidanceMassSum = avoider->mass + avoidee->mass;
		const float avoideeMassScale = avoideeMobile? (avoidee->mass / avoidanceMassSum): 1.0f;
		const float avoideeDistSq = avoideeVector.SqLength();
		const float avoideeDist   = math::sqrt(avoideeDistSq) + 0.01f;

		// do not bother steering around idling MOBILE objects
		// (since collision handling will just push them aside)
		if (avoideeMobile && avoideeMovable) {
			if (!avoiderMD->avoidMobilesOnPath || (!avoidee->IsMoving() && avoidee->allyteam == avoider->allyteam))
				continue;
		}

		// ignore objects that are more than this many degrees off-center from us
		// NOTE:
		//   if MAX_AVOIDEE_COSINE is too small, then this condition can be true
		//   one frame and false the next (after avoider has turned) causing the
		//   avoidance vector to oscillate --> units with turnInPlace = true will
		//   slow to a crawl as a result
		if (avoider->frontdir.dot(-(avoideeVector / avoideeDist)) < MAX_AVOIDEE_COSINE)
			continue;

		if (avoideeDistSq >= Square(std::max(currentSpeed, 1.0f) * GAME_SPEED + avoidanceRadiusSum))
			continue;
		if (avoideeDistSq >= avoider->pos.SqDistance2D(goalPos))
			continue;

		// if object and unit in relative motion are closing in on one another
		// (or not yet fully apart), then the object is on the path of the unit
		// and they are not collided
		if (DEBUG_DRAWING_ENABLED) {
			if (selectedUnitsHandler.selectedUnits.find(owner->id) != selectedUnitsHandler.selectedUnits.end())
				geometricObjects->AddLine(avoider->pos + (UpVector * 20.0f), avoidee->pos + (UpVector * 20.0f), 3, 1, 4);
		}

		float avoiderTurnSign = -Sign(avoidee->pos.dot(avoider->rightdir) - avoider->pos.dot(avoider->rightdir));
		float avoideeTurnSign = -Sign(avoider->pos.dot(avoidee->rightdir) - avoidee->pos.dot(avoidee->rightdir));

		// for mobile units, avoidance-response is modulated by angle
		// between avoidee's and avoider's frontdir such that maximal
		// avoidance occurs when they are anti-parallel
		const float avoidanceCosAngle = Clamp(avoider->frontdir.dot(avoidee->frontdir), -1.0f, 1.0f);
		const float avoidanceResponse = (1.0f - avoidanceCosAngle * int(avoideeMobile)) + 0.1f;
		const float avoidanceFallOff  = (1.0f - std::min(1.0f, avoideeDist / (5.0f * avoidanceRadiusSum)));

		// if parties are anti-parallel, it is always more efficient for
		// both to turn in the same local-space direction (either R/R or
		// L/L depending on relative object positions) but there exists
		// a range of orientations for which the signs are not equal
		//
		// (this is also true for the parallel situation, but there the
		// degeneracy only occurs when one of the parties is behind the
		// other and can be ignored)
		if (avoidanceCosAngle < 0.0f)
			avoiderTurnSign = std::max(avoiderTurnSign, avoideeTurnSign);

		avoidanceDir = avoider->rightdir * AVOIDER_DIR_WEIGHT * avoiderTurnSign;
		avoidanceVec += (avoidanceDir * avoidanceResponse * avoidanceFallOff * avoideeMassScale);
	}


	// use a weighted combination of the desired- and the avoidance-directions
	// also linearly smooth it using the vector calculated the previous frame
	avoidanceDir = (mix(desiredDir, avoidanceVec, DESIRED_DIR_WEIGHT)).SafeNormalize();
	avoidanceDir = (mix(avoidanceDir, lastAvoidanceDir, LAST_DIR_MIX_ALPHA)).SafeNormalize();

	if (DEBUG_DRAWING_ENABLED) {
		if (selectedUnitsHandler.selectedUnits.find(owner->id) != selectedUnitsHandler.selectedUnits.end()) {
			const float3 p0 = owner->pos + (    UpVector * 20.0f);
			const float3 p1 =         p0 + (avoidanceVec * 40.0f);
			const float3 p2 =         p0 + (avoidanceDir * 40.0f);

			const int avFigGroupID = geometricObjects->AddLine(p0, p1, 8.0f, 1, 4);
			const int adFigGroupID = geometricObjects->AddLine(p0, p2, 8.0f, 1, 4);

			geometricObjects->SetColor(avFigGroupID, 1, 0.3f, 0.3f, 0.6f);
			geometricObjects->SetColor(adFigGroupID, 1, 0.3f, 0.3f, 0.6f);
		}
	}

	return (lastAvoidanceDir = avoidanceDir);
}

}