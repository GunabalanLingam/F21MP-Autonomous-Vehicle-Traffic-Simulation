using UnityEngine;
using UnityEngine.AI;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(NavMeshAgent))]
public class Vehicle : NavAgent
{
    private NavSection currentNavSection;
    private NavConnection currentOutConenction;


    public Transform front;

    public virtual void Initialize(NavSection navSection, NavConnection destination)
    {
        currentNavSection = navSection;
        RegisterVehicle(currentNavSection, true);
        currentOutConenction = destination;
        agent.enabled = true;
        speed = TrafficSystem.Instance.GetAgentSpeedFromKPH(Mathf.Min(navSection.speedLimit, maxSpeed));
        agent.speed = speed;
        agent.destination = destination.transform.position;
    }

    public virtual void RegisterVehicle(NavSection section, bool addToSection)
    {
        if (addToSection)
            currentNavSection.StoreVehicle(this, true);
        else
            currentNavSection.StoreVehicle(this, false);
    }


    public override void Update()
    {

        if (agent.isOnNavMesh)
        {
            isBlocked = CheckForObstacles();
        }
        if(isBlocked)
            agent.velocity= Vector3.zero;   
        base.Update();

    }

    public override bool CheckStop()
    {
        return isBlocked || isWaiting;
    }


    public override void OnTriggerEnter(Collider other)
    {
        base.OnTriggerEnter(other);

        if (other.tag == "RoadConnection")
        {
            NavConnection connection = other.GetComponent<NavConnection>();
            if (connection.navSection != currentNavSection)
                ChangeRoad(connection);
        }
    }

    public bool isBlocked;

    public float blockDistamce = 3f;
 

    private bool CheckForObstacles()
    {
        Vector3 forward = transform.TransformDirection(Vector3.forward);
        RaycastHit hit;
        if (Physics.Raycast(front.position, forward, out hit))
        {
            if (Vector3.Distance(front.position, hit.point) < blockDistamce)
            {
                if (hit.transform.tag == "Gib" || hit.transform.tag == "Unit"||hit.transform.tag == "Player")
                    return true;
            }
            return false;
        }
        return false;
    }

    private void ChangeRoad(NavConnection newConnection)
    {
        RegisterVehicle(currentNavSection, false);
        speed = TrafficSystem.Instance.GetAgentSpeedFromKPH(Mathf.Min(newConnection.navSection.speedLimit, maxSpeed));
        agent.speed = speed;
        currentNavSection = newConnection.navSection;
        RegisterVehicle(currentNavSection, true);
        currentOutConenction = newConnection.GetOutConnection();
        if (currentOutConenction != null)
            agent.destination = currentOutConenction.transform.position;
    }
    public void OnDrawGizmos()
    {
        if (TrafficSystem.Instance.GizmoOnSceneView)
        {
            DrawPathGizmos();
            DrawObstacleDetectionRay();
        }
    }

    private void DrawPathGizmos()
    {
        Gizmos.color = CheckStop() ? Color.gray : Color.white;
        if (agent.hasPath)
        {
            Gizmos.DrawWireSphere(agent.destination, 0.1f);
            for (int i = 0; i < agent.path.corners.Length - 1; i++)
            {
                Gizmos.DrawLine(agent.path.corners[i], agent.path.corners[i + 1]);
            }
        }
    }

    private void DrawObstacleDetectionRay()
    {
        Gizmos.color = isBlocked ? Color.red : Color.green;
        Vector3 end = front.TransformPoint(Vector3.forward * blockDistamce);
        Gizmos.DrawLine(front.position, end);
    }
}


