using UnityEngine;

public class PathManager
{
    static PathManager instance;
    private PathManager() { }

    public static PathManager Instance()
    {
        if (instance == null)
        {
            instance = new PathManager();
        }
        return instance;
    }

    public void DeletePath(int pathID)
    {
    }
    public bool IsFinalized()
    {
        return true;
    }
    public int RequestPath(Unit obj, Vector3 sourcePoint, Vector3 targetPoint, float radius, bool synced)
    {
        if (!IsFinalized())
        {
            return 0;
        }
        return 0;
    }
    public bool UpdatePath(Unit owner, int pathID)
    {
        return true;
    }
    public Vector3 NextWayPoint(Unit obj, int pathID, int numRetries, Vector3 point, float radius, bool synced)
    {
        return Vector3.zero;
    }
}