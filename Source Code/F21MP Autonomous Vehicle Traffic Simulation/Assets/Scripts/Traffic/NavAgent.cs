using UnityEngine;
using UnityEngine.AI;

[RequireComponent(typeof(NavMeshAgent))]
public class NavAgent : MonoBehaviour
{
    public NavMeshAgent agent;

    public Animator animationController;

    public TrafficType type = TrafficType.Pedestrian;
    public int maxSpeed = 20;

    Vector3 lastPosition;

    public void Start()
    {
        agent = GetComponent<NavMeshAgent>();

        lastPosition = transform.position;
    }
  

    public void Initialize()
    {
        agent.enabled = true;
        speed = TrafficSystem.Instance.GetAgentSpeedFromKPH(maxSpeed);
        agent.speed = speed;
        SetDestination();
    }

    private void SetDestination()
    {
        agentDestination = TrafficSystem.Instance.GetPedAgentDestination();
        if (agentDestination)
            agent.SetDestination(agentDestination.position);
    }

    float m_Speed;
    public float speed
    {
        get { return m_Speed; }
        set { m_Speed = value; }
    }
    Transform agentDestination;

    public virtual void Update()
    {
        UpdateMovement();
        if (animationController)
        {
            animationController.SetFloat("Movement", GetMovementSpeed());
        }

    }

    private void UpdateMovement()
    {
        if (!agent.isOnNavMesh)
            return;

        if (isWaiting || CheckStopConditions())
            agent.velocity = Vector3.zero;
        else
            CheckDestinationReached();

        CheckWaitZone();
    }
    private float GetMovementSpeed()
    {
        float speed = (transform.position - lastPosition).magnitude / Time.deltaTime;
        lastPosition = transform.position;
        return speed;
    }
    private void CheckDestinationReached()
    {
        if (agentDestination && Vector3.Distance(transform.position, agentDestination.position) < 1f)
        {
            SetDestination();
        }
    }


    private bool CheckStopConditions()
    {
        return isWaiting && currentWaitZone && !currentWaitZone.canPass;
    }


    public virtual void OnTriggerEnter(Collider col)
    {
        if (col.tag == "WaitZone")
        {
            WaitZone waitZone = col.GetComponent<WaitZone>();
            if (waitZone.type == type)
            {
                if (type == TrafficType.Pedestrian)
                {
                    if (CheckOppositeWAitZone(waitZone))
                        return;
                }
                currentWaitZone = waitZone;
                if (!waitZone.canPass)
                    isWaiting = true;
            }
        }
    }

    private bool CheckOppositeWAitZone(WaitZone waitZone)
    {
        if (waitZone.opposite)
        {
            if (waitZone.opposite == currentWaitZone)
                return true;
        }
        return false;
    }


    WaitZone currentWaitZone;

    public bool isWaiting;

    private void CheckWaitZone()
    {
        if (isWaiting)
        {
            if (currentWaitZone)
                isWaiting = !currentWaitZone.canPass;
        }
    }

    public virtual bool CheckStop()
    {
        if (isWaiting)
            return true;
        return false;
    }

}

public enum TrafficType { Pedestrian, Vehicle }

