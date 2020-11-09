using System.Collections.Generic;
using UnityEngine;

public class QuadFieldQuery
{
    public List<int> quads = new List<int>();
    public List<Unit> units = new List<Unit>();
}

public class QuadField
{
    private static QuadField instance;
    List<Unit> units = new List<Unit>();
    private QuadField()
    {
    }
    public static QuadField Instance()
    {
        if (instance == null)
        {
            instance = new QuadField();
        }
        return instance;
    }
    public void WorldPosToQuadField(Vector3 p, out int x, out int y)
    {
        x = 0;
        y = 0;
    }
    public void GetSolidsExact(QuadFieldQuery qfq, Vector3 pos, float radius)
    {
        foreach (var unit in units)
        {
            if ((pos - unit.pos).sqrMagnitude >= PathMathUtils.Square(radius + unit.radius))
            {
                continue;
            }
            qfq.units.Add(unit);
        }
    }
    public void GetUnitsExact(QuadFieldQuery qfq, Vector3 pos, float radius, bool spherical = true)
    {
    }
    public void MovedUnit(Unit unit)
    {
    }
}