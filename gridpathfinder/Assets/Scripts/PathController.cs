using UnityEngine;

public class PathController
{
    Unit owner;
    Vector3 realGoalPos; // where <owner> ultimately wants to go (its final waypoint)
    Vector3 tempGoalPos; // where <owner> currently wants to go (its next waypoint)
    public PathController(Unit owner)
    {
        this.owner = owner;
    }
    public void SetTempGoalPosition(int pathID, Vector3 pos) { realGoalPos = pos; }
    public void SetRealGoalPosition(int pathID, Vector3 pos) { tempGoalPos = pos; }
    public float GetDeltaSpeed(int pathID, float targetSpeed, float currentSpeed, float maxAccRat, float maxDecRate, bool wantReverse, bool isReversing)
    {
        return 0;
    }
    public int GetDeltaHeading(int pathID, int newHeading, int oldHeading, float maxTurnSpeed, float maxTurnAccel, float turnBrakeDist, ref float curTurnSpeedPtr)
    {
        float curTurnSpeed = curTurnSpeedPtr;
        float absTurnSpeed = Mathf.Abs(curTurnSpeed);
        // negative delta represents a RH turn, positive a LH turn
        // add lookahead term to avoid overshooting target heading
        // note that turnBrakeDist is always positive
        int brakeDistFactor = (absTurnSpeed >= maxTurnAccel) ? 1 : 0;
        int stopTurnHeading = oldHeading + (int)(turnBrakeDist * PathMathUtils.Sign(curTurnSpeed) * brakeDistFactor);
        int curDeltaHeading = newHeading - stopTurnHeading;
        if (brakeDistFactor == 0)
        {
            curTurnSpeed = (PathMathUtils.Sign(curDeltaHeading) * Mathf.Min(Mathf.Abs(curDeltaHeading * 1.0f), maxTurnAccel));
        }
        else
        {
            curTurnSpeed += (PathMathUtils.Sign(curDeltaHeading) * Mathf.Min(Mathf.Abs(curDeltaHeading * 1.0f), maxTurnAccel));
        }
        // less realistic to nullify speed in air, but saves headaches
        // (high-turnrate units leaving the ground do not behave well)
        return (int)(curTurnSpeedPtr = Mathf.Clamp(curTurnSpeed, -maxTurnSpeed, maxTurnSpeed));
    }
}