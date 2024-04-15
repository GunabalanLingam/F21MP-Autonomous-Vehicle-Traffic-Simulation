using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEditor;
using UnityEngine;
using static Junction;


public class TrafficSystem : MonoBehaviour
{
    private static TrafficSystem instance;

    public static TrafficSystem Instance
    {
        get => instance ? instance : instance = FindObjectOfType<TrafficSystem>();
    }


    public List<GameObject> buildingPrefabs = new List<GameObject>();
    public List<Transform > buildingSpawnPoints = new List<Transform>();


    public bool GizmoOnSceneView = true;
    public List<GameObject> carPrefabs = new List<GameObject>();
    public List<GameObject> pedestrianPrefabs = new List<GameObject>();

    public Transform pool;
    public bool spawnOnStart = true;
    public int maxRoadVehicles = 100;
    public int maxPedestrians = 100;


    private int vehicleSpawnCount;
    private int pedSpawnCount;


    private List<Road> roads = new List<Road>();

    private void Start()
    {

        roads = new List<Road>(FindObjectsOfType<Road>());
        if (spawnOnStart)
        {
            InitializeScene();
        }
    }

    private void InitializeScene()
    {
        SpawnAllBuildings();
        RepeatAction(SpawnVehicle, maxRoadVehicles);
        RepeatAction(SpawnPedestrian, maxPedestrians);
    }

    private void RepeatAction(System.Action<bool> action, int count)
    {
        for (int i = 0; i < count; i++)
        {
            action.Invoke(true);
        }
    }

    private void Update()
    {
        if (Input.GetKeyUp(KeyCode.P))
            SpawnPedestrian(true);
        if (Input.GetKeyUp(KeyCode.V))
            SpawnVehicle(true);
    }

    private void SpawnAllBuildings()
    {
        foreach (Transform spawnPoint in buildingSpawnPoints)
        {
            var prefab = buildingPrefabs[Random.Range(0, buildingPrefabs.Count)];
            Instantiate(prefab, spawnPoint.position, spawnPoint.rotation);
        }
    }
    private void SpawnVehicle(bool reset)
    {
        if (reset)
            vehicleSpawnCount = 0;

        int index = Random.Range(0, roads.Count);
        if (roads[index].GetVehicle(out VehicleSpawn spawn))
        {
            var vehiclePrefab = carPrefabs[Random.Range(0, carPrefabs.Count)];
            var newVehicle = Instantiate(vehiclePrefab, spawn.spawn.position, spawn.spawn.rotation, pool).GetComponent<Vehicle>();
            newVehicle.Initialize(roads[index], spawn.destination);
        }
        else if (++vehicleSpawnCount < roads.Count)
        {
            SpawnVehicle(false);
        }
    }

    private void SpawnPedestrian(bool reset)
    {
        if (reset)
            pedSpawnCount = 0;

        int index = UnityEngine.Random.Range(0, roads.Count);
        if (roads[index].GetPed(out Transform spawn))
        {
            var pedestrianPrefab = pedestrianPrefabs[Random.Range(0, pedestrianPrefabs.Count)];
            var newAgent = Instantiate(pedestrianPrefab, spawn.position, spawn.rotation, pool).GetComponent<NavAgent>();
            newAgent.Initialize();
        }
        else if (++pedSpawnCount < roads.Count)
        {
            SpawnPedestrian(false);
        }
    }

    public Transform GetPedAgentDestination()
    {
        int index = UnityEngine.Random.Range(0, roads.Count);
        Road road = roads[index];
        Transform destination;
        if (!road.GetPed(out destination))
        {
            return GetPedAgentDestination();
        }
        return destination;
    }

    public float GetAgentSpeedFromKPH(int kph)
    {
        return kph ;
    }

    private void OnDrawGizmos()
    {
        if (GizmoOnSceneView)
        {
            Gizmos.color = Color.red;
            foreach (var spawn in buildingSpawnPoints)
            {
                Gizmos.DrawSphere(spawn.position, 0.5f);
            }
        }
    }
}

