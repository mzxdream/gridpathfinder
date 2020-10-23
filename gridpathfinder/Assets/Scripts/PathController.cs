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
}