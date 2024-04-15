
using System;
using System.Collections.Generic;
using UnityEngine;


public class NavSection : MonoBehaviour
{
    [Header("Nav Section")]
    public VehicleSpawn[] vehicleSpawns;
    public NavConnection[] connections;
    public int speedLimit = 20;
    private List<Vehicle> currentVehicles = new List<Vehicle>();

    public virtual void Start()
    {
        InitializeConnections();
    }

    private void InitializeConnections()
    {
        foreach (var connection in connections)
        {
            connection.navSection = this;
        }
    }

    public bool GetVehicle(out VehicleSpawn spawn)
    {
        spawn = null;
        if (currentVehicles.Count == 0 && vehicleSpawns.Length > 0)
        {
            int index = UnityEngine.Random.Range(0, vehicleSpawns.Length);
            spawn = vehicleSpawns[index];
            return true;
        }
        return false;
    }

    public void StoreVehicle(Vehicle vehicle, bool addToSection)
    {
        if (addToSection)
        {
            currentVehicles.Add(vehicle);
        }
        else
        {
            if (currentVehicles.Remove(vehicle) == false)
            {
                Debug.LogWarning($"Attempted to remove non-existing vehicle from: {gameObject.name}");
            }
        }
    }


}

[Serializable]
public class VehicleSpawn
{
    public Transform spawn;
    public NavConnection destination;
}

