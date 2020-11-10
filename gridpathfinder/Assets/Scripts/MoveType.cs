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
        var oldSpeed = Mathf.Abs(Vector3.Dot(oldSpeedVector, flatFrontDir));
        var newSpeed = Mathf.Abs(Vector3.Dot(newSpeedVector, flatFrontDir));
        if (newSpeedVector != Vector3.zero)
        {
            owner.SetVelocityAndSpeed(newSpeedVector);
            owner.Move(owner.speed, true);
            if (!owner.moveDef.TestMoveSquare(owner, owner.pos, owner.speed, true, false, true))
            {
                bool updatePos = false;
                for (int n = 1; n <= Game.SQUARE_SIZE; n++)
                {
                    if (!updatePos && (updatePos = owner.moveDef.TestMoveSquare(owner, owner.pos + owner.rightdir * n, owner.speed, true, false, true)))
                    {
                        owner.Move(owner.pos + owner.rightdir * n, false);
                        break;
                    }
                    if (!updatePos && (updatePos = owner.moveDef.TestMoveSquare(owner, owner.pos - owner.rightdir * n, owner.speed, true, false, true)))
                    {
                        owner.Move(owner.pos - owner.rightdir * n, false);
                        break;
                    }
                }
                if (!updatePos)
                {
                    owner.Move(owner.pos - newSpeedVector, false);
                }
            }
        }
        reversing = (Vector3.Dot(newSpeedVector, flatFrontDir) < 0.0f);
        currentSpeed = newSpeed;
        deltaSpeed = 0.0f;
    }
    float FootPrintRadius(int xs, int zs, float s)
    {
        return Mathf.Sqrt(xs * xs + zs * zs) * 0.5f * Game.SQUARE_SIZE * s;
    }
    public void HandleObjectCollisions()
    {
        var collider = owner;
        var colliderUD = collider.unitDef;
        var colliderMD = collider.moveDef;
        var colliderSpeed = collider.speedw;
        var colliderRadius = FootPrintRadius(colliderMD.xsize, colliderMD.zsize, 0.75f);
        HandleUnitCollisions(collider, colliderSpeed, colliderRadius, colliderUD, colliderMD);
        bool squareChange = (Game.GetSquare(owner.pos + owner.speed) != Game.GetSquare(owner.pos));
        //if (squareChange)
        {
            HandleStaticObjectCollision(owner, owner, owner.moveDef, colliderRadius, 0.0f, Vector3.zero, true, false, true);
        }
    }
    public void HandleUnitCollisions(Unit collider, float colliderSpeed, float colliderRadius, UnitDef colliderUD, MoveDef colliderMD)
    {
        float searchRadius = colliderSpeed + (colliderRadius * 2.0f);
        QuadFieldQuery qfQuery = new QuadFieldQuery();
        QuadField.Instance().GetUnitsExact(qfQuery, collider.pos, searchRadius);
        int dirSign = (int)PathMathUtils.Sign(!reversing ? 1f : 0f);
        Vector3 crushImpulse = collider.speed * collider.unitDef.mass * dirSign;
        foreach (var collidee in qfQuery.units)
        {
            if (collidee == collider) continue;
            UnitDef collideeUD = collidee.unitDef;
            MoveDef collideeMD = collidee.moveDef;
            bool colliderMobile = true;
            bool collideeMobile = true;// (collideeMD != NULL); // maybe true
            bool unloadingCollidee = false;
            bool unloadingCollider = false;
            float collideeSpeed = collidee.speedw;
            float collideeRadius = FootPrintRadius(collideeMD.xsize, collideeMD.zsize, 0.75f);
            Vector3 separationVector = collider.pos - collidee.pos;
            float separationMinDistSq = (colliderRadius + collideeRadius) * (colliderRadius + collideeRadius);
            if ((separationVector.sqrMagnitude - separationMinDistSq) > 0.01f)
                continue;
            bool pushCollider = colliderMobile;
            bool pushCollidee = collideeMobile;
            //bool crushCollidee = false;
            bool alliedCollision = false;
            //teamHandler->Ally(collider->allyteam, collidee->allyteam) &&
            //teamHandler->Ally(collidee->allyteam, collider->allyteam);
            bool collideeYields = collider.IsMoving && !collidee.IsMoving;
            bool ignoreCollidee = (collideeYields && alliedCollision);
            //crushCollidee |= !alliedCollision;
            //crushCollidee &= ((colliderSpeed * collider->mass) > (collideeSpeed * collidee->mass));

            //if (crushCollidee && !CMoveMath::CrushResistant(*colliderMD, collidee))
            //    collidee->Kill(collider, crushImpulse, true);

            //if (pathController.IgnoreCollision(collider, collidee))
            //    continue;
            if (collideeMobile)
            {
                var gmt = collidee.moveType;
                if (PathMathUtils.SqrDistance2D(collider.moveType.goalPos, collidee.moveType.goalPos) < Mathf.PI * Mathf.PI) //
                {
                    if (collider.IsMoving && collider.moveType.progressState == MoveType.ProgressState.Active)
                    {
                        if (collidee.moveType.progressState == MoveType.ProgressState.Done)
                        {
                            if (!collidee.IsMoving)
                            {
                                atEndOfPath = true; atGoal = true;
                            }
                            // We're in a traffic jam so ignore current way point and go directly to the next one
                        }
                        else if (collidee.moveType.progressState == MoveType.ProgressState.Active && gmt.currWayPoint == nextWayPoint)
                        {
                            currWayPoint.y = -1.0f;//重新选择下一个路点，交通繁忙
                        }
                    }
                }
            }
            pushCollider = pushCollider && (alliedCollision || !collider.blockEnemyPushing);
            pushCollidee = pushCollidee && (alliedCollision || !collidee.blockEnemyPushing);
            //pushCollider = pushCollider && !collider.moveType.IsPushResistant();
            //pushCollidee = pushCollidee && !collidee->moveType->IsPushResistant();
            if (!collideeMobile || (!pushCollider && !pushCollidee))
            {
                // building (always axis-aligned, possibly has a yardmap)
                // or semi-static collidee that should be handled as such
                // this also handles two mutually push-resistant parties!
                HandleStaticObjectCollision(
                    collider,
                    collidee,
                    colliderMD,
                    colliderRadius,
                    collideeRadius,
                    separationVector,
                    (!atEndOfPath && !atGoal),
                    false,
                    false);
                continue;
            }
            float colliderRelRadius = colliderRadius / (colliderRadius + collideeRadius);
            float collideeRelRadius = collideeRadius / (colliderRadius + collideeRadius);
            float collisionRadiusSum = (colliderRadius + collideeRadius);
            float sepDistance = separationVector.magnitude + 0.1f;
            float penDistance = Mathf.Max(collisionRadiusSum - sepDistance, 1.0f);
            float sepResponse = Mathf.Min(Game.SQUARE_SIZE * 2.0f, penDistance * 0.5f);
            Vector3 sepDirection = (separationVector / sepDistance);
            Vector3 colResponseVec = new Vector3(sepDirection.x, 0, sepDirection.z) * sepResponse;
            float
                m1 = collider.unitDef.mass,
                m2 = collidee.unitDef.mass,
                v1 = Mathf.Max(1.0f, colliderSpeed),
                v2 = Mathf.Max(1.0f, collideeSpeed),
                c1 = 1.0f + (1.0f - Mathf.Abs(Vector3.Dot(collider.frontdir, -sepDirection))) * 5.0f,
                c2 = 1.0f + (1.0f - Mathf.Abs(Vector3.Dot(collidee.frontdir, sepDirection))) * 5.0f,
                s1 = m1 * v1 * c1,
                s2 = m2 * v2 * c2,
                r1 = s1 / (s1 + s2 + 1.0f),
                r2 = s2 / (s1 + s2 + 1.0f);
            // far from a realistic treatment, but works
            float colliderMassScale = Mathf.Clamp(1.0f - r1, 0.01f, 0.99f);
            float collideeMassScale = Mathf.Clamp(1.0f - r2, 0.01f, 0.99f);
            float colliderSlideSign = Mathf.Sign(Vector3.Dot(separationVector, collider.rightdir));
            float collideeSlideSign = Mathf.Sign(Vector3.Dot(-separationVector, collidee.rightdir));
            Vector3 colliderPushVec = colResponseVec * colliderMassScale * (int)(!ignoreCollidee ? 1 : 0);
            Vector3 collideePushVec = -colResponseVec * collideeMassScale;
            Vector3 colliderSlideVec = collider.rightdir * colliderSlideSign * (1.0f / penDistance) * r2;
            Vector3 collideeSlideVec = collidee.rightdir * collideeSlideSign * (1.0f / penDistance) * r1;
            Vector3 colliderMoveVec = colliderPushVec + colliderSlideVec;
            Vector3 collideeMoveVec = collideePushVec + collideeSlideVec;
            if ((pushCollider || !pushCollidee) && colliderMobile)
            {
                if (colliderMD.TestMoveSquare(collider, collider.pos + colliderMoveVec, colliderMoveVec))
                {
                    collider.Move(colliderMoveVec, true);
                }
            }
            if ((pushCollidee || !pushCollider) && collideeMobile)
            {
                if (collideeMD.TestMoveSquare(collidee, collidee.pos + collideeMoveVec, collideeMoveVec))
                {
                    collidee.Move(collideeMoveVec, true);
                }
            }
        }
    }
    public void HandleStaticObjectCollision(Unit collider, Unit collidee, MoveDef colliderMD, float colliderRadius, float collideeRadius, Vector3 separationVector, bool canRequestPath, bool checkYardMap, bool checkTerrain)
    {
        if (checkTerrain && !collider.IsMoving)
        {
            return;
        }
        bool wantRequestPath = false;
        if (checkYardMap || checkTerrain)
        {
            int xmid = (int)(collider.pos.x + collider.speed.x) / Game.SQUARE_SIZE;
            int zmid = (int)(collider.pos.z + collider.speed.z) / Game.SQUARE_SIZE;
            int xsh = colliderMD.xsizeh * ((checkYardMap || (checkTerrain && colliderMD.allowTerrainCollisions)) ? 1 : 0);
            int zsh = colliderMD.zsizeh * ((checkYardMap || (checkTerrain && colliderMD.allowTerrainCollisions)) ? 1 : 0);
            int xmin = Mathf.Min(-1, -xsh);
            int xmax = Mathf.Max(1, xsh);
            int zmin = Mathf.Min(-1, -zsh);
            int zmax = Mathf.Max(1, zsh);
            Vector3 strafeVec = Vector3.zero;
            Vector3 bounceVec = Vector3.zero;
            Vector3 sqrSumPosition = Vector3.zero; // .y is always 0
            Vector3 sqrPenDistance = Vector3.zero; // .x = sum, .y = count
            Vector3 speed2D = new Vector3(collider.speed.x, 0, collider.speed.z).normalized;
            for (int z = zmin; z <= zmax; z++)
            {
                for (int x = xmin; x <= xmax; x++)
                {
                    int xabs = xmid + x;
                    int zabs = zmid + z;
                    if (checkTerrain)
                    {
                        //if (CMoveMath::GetPosSpeedMod(*colliderMD, xabs, zabs, speed2D) > 0.01f)
                        //    continue;
                    }
                    else
                    {
                        if ((PathMathUtils.SquareIsBlocked(colliderMD, xabs, zabs, collider) & (int)PathMathUtils.BlockTypes.BLOCK_STRUCTURE) == 0)
                            continue;
                    }
                    Vector3 squarePos = new Vector3(xabs * Game.SQUARE_SIZE + (Game.SQUARE_SIZE >> 1), collider.pos.y, zabs * Game.SQUARE_SIZE + (Game.SQUARE_SIZE >> 1));
                    Vector3 squareVec = collider.pos - squarePos;
                    if (Vector3.Dot(squareVec, collider.speed) > 0.0f)
                        continue;
                    // RHS magic constant is the radius of a square (sqrt(2*(SQUARE_SIZE>>1)*(SQUARE_SIZE>>1)))
                    float squareColRadiusSum = colliderRadius + 5.656854249492381f;
                    float squareSepDistance = Mathf.Sqrt(PathMathUtils.SqrDistance2D(collider.pos, squarePos)) + 0.1f;
                    float squarePenDistance = Mathf.Min(squareSepDistance - squareColRadiusSum, 0.0f);
                    bounceVec += (collider.rightdir * Vector3.Dot(collider.rightdir, squareVec / squareSepDistance));
                    sqrPenDistance += new Vector3(squarePenDistance, 0, 1.0f);
                    sqrSumPosition += new Vector3(squarePos.x, 0, squarePos.z);
                }
            }
            if (sqrPenDistance.y > 0.0f)
            {
                sqrSumPosition.x /= sqrPenDistance.z;
                sqrSumPosition.z /= sqrPenDistance.z;
                sqrPenDistance.x /= sqrPenDistance.z;
                float strafeSign = -PathMathUtils.Sign(Vector3.Dot(sqrSumPosition, collider.rightdir) - Vector3.Dot(collider.pos, collider.rightdir));
                float strafeScale = Mathf.Min(Mathf.Max(currentSpeed * 0.0f, maxSpeed), Mathf.Max(0.1f, -sqrPenDistance.x * 0.5f));
                float bounceScale = Mathf.Min(Mathf.Max(currentSpeed * 0.0f, maxSpeed), Mathf.Max(0.1f, -sqrPenDistance.x * 0.5f));
                float fpsStrafeScale = (strafeScale / (strafeScale + bounceScale)) * maxSpeed;
                float fpsBounceScale = (bounceScale / (strafeScale + bounceScale)) * maxSpeed;
                strafeVec = collider.rightdir * strafeSign;
                strafeVec = new Vector3(strafeVec.x, 0, strafeVec.z).normalized * strafeScale;
                bounceVec = new Vector3(bounceVec.x, 0, bounceVec.z).normalized * bounceScale;
                // if checkTerrain is true, test only the center square
                if (colliderMD.TestMoveSquare(collider, collider.pos + strafeVec + bounceVec, collider.speed, checkTerrain, checkYardMap, checkTerrain))
                {
                    collider.Move(strafeVec + bounceVec, true);
                }
                else
                {
                    collider.Move(oldPos - collider.pos, wantRequestPath = true);
                }
            }
            wantRequestPath = (strafeVec + bounceVec != Vector3.zero);
        }
        else
        {
            float colRadiusSum = colliderRadius + collideeRadius;
            float sepDistance = separationVector.magnitude + 0.1f;
            float penDistance = Mathf.Min(sepDistance - colRadiusSum, 0.0f);
            float colSlideSign = -PathMathUtils.Sign(Vector3.Dot(collidee.pos, collider.rightdir) - Vector3.Dot(collider.pos, collider.rightdir));
            float strafeScale = Mathf.Min(currentSpeed, Mathf.Max(0.0f, -penDistance * 0.5f));
            float bounceScale = Mathf.Min(currentSpeed, Mathf.Max(0.0f, -penDistance)) * (1 - (checkYardMap ? 1 : 0));
            Vector3 strafeVec = (collider.rightdir * colSlideSign) * strafeScale;
            Vector3 bounceVec = (separationVector / sepDistance) * bounceScale;
            if (colliderMD.TestMoveSquare(collider, collider.pos + strafeVec + bounceVec, collider.speed, true, true, true))
            {
                collider.Move(strafeVec + bounceVec, true);
            }
            else
            {
                if (Vector3.Dot(collider.frontdir, separationVector) < 0.25f)
                {
                    wantRequestPath = true;
                    collider.Move(oldPos - collider.pos, true);
                }
            }
            // same here
            wantRequestPath |= (penDistance < 0.0f);
        }
        if (canRequestPath && wantRequestPath)
        {
            ReRequestPath(false);
        }
    }
    public bool OwnerMoved(int oldHeading, Vector3 posDif, Vector3 cmpEps)
    {
        //    if (posDif.equals(ZeroVector, cmpEps))
        if (posDif.x < cmpEps.x && posDif.y < cmpEps.y && posDif.z < cmpEps.z)
        {
            owner.SetVelocityAndSpeed(Vector3.zero);
            idling = true;
            idling &= (currWayPoint.y != -1.0f && nextWayPoint.y != -1.0f);
            idling &= (Mathf.Abs(owner.heading - oldHeading) < turnRate);
            return false;
        }
        oldPos = owner.pos;
        var ffd = flatFrontDir * posDif.sqrMagnitude * 0.5f;
        var wpd = waypointDir * (!reversing ? 1 : -1);
        idling = true;
        idling &= (PathMathUtils.Square(currWayPointDist - prevWayPointDist) < Vector3.Dot(ffd, wpd));
        return true;
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
        if (WantToStop())
            return Vector3.zero;
        // Speed-optimizer. Reduces the times this system is run.
        //if (Game.frameNum < nextObstacleAvoidanceFrame)
        //    return lastAvoidanceDir;
        Vector3 avoidanceVec = Vector3.zero;
        Vector3 avoidanceDir = desiredDir;
        lastAvoidanceDir = desiredDir;
        nextObstacleAvoidanceFrame = Game.frameNum + 1;
        var avoider = owner;
        var avoiderMD = avoider.moveDef;
        if (Vector3.Dot(avoider.frontdir, desiredDir) < 0.0f)
            return lastAvoidanceDir;
        const float AVOIDER_DIR_WEIGHT = 1.0f;
        const float DESIRED_DIR_WEIGHT = 0.5f;
        float MAX_AVOIDEE_COSINE = Mathf.Cos(120.0f * 0.017453292519943295f);
        const float LAST_DIR_MIX_ALPHA = 0.7f;
        float avoidanceRadius = Mathf.Max(currentSpeed, 1.0f) * (avoider.radius * 2.0f);
        float avoiderRadius = FootPrintRadius(avoiderMD.xsize, avoiderMD.zsize, 1.0f);
        QuadFieldQuery qfQuery = new QuadFieldQuery();
        QuadField.Instance().GetSolidsExact(qfQuery, avoider.pos, avoidanceRadius);
        foreach (var avoidee in qfQuery.units)
        {
            var avoideeMD = avoidee.moveDef;
            var avoideeUD = avoidee.unitDef;
            if (avoidee == owner)
                continue;
            bool avoideeMobile = true;
            bool avoideeMovable = true;
            Vector3 avoideeVector = (avoider.pos + avoider.speed) - (avoidee.pos + avoidee.speed);
            float avoideeRadius = FootPrintRadius(avoideeMD.xsize, avoideeMD.zsize, 1.0f);
            float avoidanceRadiusSum = avoiderRadius + avoideeRadius;
            float avoidanceMassSum = avoider.unitDef.mass + avoidee.unitDef.mass;
            float avoideeMassScale = avoideeMobile ? (avoidee.unitDef.mass / avoidanceMassSum) : 1.0f;
            float avoideeDistSq = avoideeVector.sqrMagnitude;
            float avoideeDist = Mathf.Sqrt(avoideeDistSq) + 0.01f;
            if (avoideeMobile && avoideeMovable)
            {
                if (!avoiderMD.avoidMobilesOnPath || (!avoidee.IsMoving && avoidee.allyteam == avoider.allyteam))
                {
                    continue;
                }
            }
            if (Vector3.Dot(avoider.frontdir, -(avoideeVector / avoideeDist)) < MAX_AVOIDEE_COSINE)
                continue;
            if (avoideeDistSq >= PathMathUtils.Square(Mathf.Max(currentSpeed, 1.0f) * Game.GAME_SPEED + avoidanceRadiusSum))
                continue;
            if (avoideeDistSq >= PathMathUtils.SqrDistance2D(avoider.pos, goalPos))
                continue;
            float avoiderTurnSign = -PathMathUtils.Sign(Vector3.Dot(avoidee.pos, avoider.rightdir) - Vector3.Dot(avoider.pos, avoider.rightdir));
            float avoideeTurnSign = -PathMathUtils.Sign(Vector3.Dot(avoider.pos, avoidee.rightdir) - Vector3.Dot(avoidee.pos, avoidee.rightdir));
            float avoidanceCosAngle = Mathf.Clamp(Vector3.Dot(avoider.frontdir, avoidee.frontdir), -1.0f, 1.0f);
            float avoidanceResponse = (1.0f - avoidanceCosAngle) + 0.1f;
            float avoidanceFallOff = (1.0f - Mathf.Min(1.0f, avoideeDist / (5.0f * avoidanceRadiusSum)));
            if (avoidanceCosAngle < 0.0f)
                avoiderTurnSign = Mathf.Max(avoiderTurnSign, avoideeTurnSign);
            avoidanceDir = avoider.rightdir * AVOIDER_DIR_WEIGHT * avoiderTurnSign;
            avoidanceVec += (avoidanceDir * avoidanceResponse * avoidanceFallOff * avoideeMassScale);
        }
        // use a weighted combination of the desired- and the avoidance-directions
        // also linearly smooth it using the vector calculated the previous frame
        avoidanceDir = (PathMathUtils.MixVec3(desiredDir, avoidanceVec, DESIRED_DIR_WEIGHT)).normalized;
        avoidanceDir = (PathMathUtils.MixVec3(avoidanceDir, lastAvoidanceDir, LAST_DIR_MIX_ALPHA)).normalized;
        return (lastAvoidanceDir = avoidanceDir);
    }
}