using UnityEngine;

public class Road : NavSection
{

    [Header("Road")]
    public Transform[] pedestrianSpawns;

    public bool isCurve;
    public bool GetPed(out Transform spawn)
    {
        spawn = null;
        int spawnCount = pedestrianSpawns.Length;

        if (spawnCount > 0)
        {
            int index = UnityEngine.Random.Range(0, spawnCount);
            spawn = pedestrianSpawns[index];
            return true;
        }

        return false;
    }

    public NavConnection GetOutConnectionRoad(int index)
    {
        if (connections != null && index >= 0 && index < connections.Length)
        {
            return connections[index];
        }
        return null;
    }
}

