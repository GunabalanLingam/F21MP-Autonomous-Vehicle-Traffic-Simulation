using Assets.Scripts;
using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class CarAgentController : Agent
{
    private const float POSITION_CHANGE_WATCHDOG = 120;

    [SerializeField]
    private List<AxleInfo> _axleInfos = null;
    [SerializeField]
    private float _maxMotorTorque = 800;
    [SerializeField]
    private float _maxSteeringAngle = 30;

    public float speed;
    public float steeringInput;
    public float throttleInput;

    private Rigidbody rigidBody;
    private Vector3 startingPosition;
    private Quaternion startingrotation;
    private List<GameObject> collectedRewards;
    private int rewardsCount;


    private void Start()
    {
        startingPosition = transform.position;
        startingrotation = transform.rotation;
        rigidBody = transform.GetComponent<Rigidbody>();
        collectedRewards = new List<GameObject>();
    }

    public override void OnEpisodeBegin()
    {
        StartCoroutine(CheckForStagnation());
        ResetPositionAndVelocity();
        ResetRewards();
    }
    private void ResetPositionAndVelocity()
    {
        rigidBody.velocity = Vector3.zero;
        transform.position = startingPosition;
        transform.rotation = startingrotation;
    }

    private void ResetRewards()
    {
        foreach (var reward in collectedRewards)
        {
            reward.SetActive(true);
        }
        collectedRewards.Clear();
    }



    private IEnumerator CheckForStagnation()
    {
        Vector3 positionBefore = transform.position;

        yield return new WaitForSeconds(POSITION_CHANGE_WATCHDOG);

        float distance = Vector3.Distance(positionBefore, transform.position);
        if (distance < 0.5f)
        {

            EndEpisode();
        }
        else
        {
            StartCoroutine(CheckForStagnation());
        }

    }
    public override void CollectObservations(VectorSensor sensor)
    {
        var relativeVelocity = transform.InverseTransformDirection(rigidBody.velocity);
        sensor.AddObservation(relativeVelocity.x);
        sensor.AddObservation(relativeVelocity.z);
        sensor.AddObservation(rigidBody.velocity.magnitude);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continuousActionsOut = actionsOut.ContinuousActions;

        continuousActionsOut[0] = 0f;
        continuousActionsOut[1] = 0f;

        continuousActionsOut[0] = Input.GetAxis("Horizontal"); 
        continuousActionsOut[1] = Input.GetAxis("Vertical");

    }
    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        Move(actionBuffers.ContinuousActions[0], actionBuffers.ContinuousActions[1]);
    }
    private void Move(float horizontal, float vertical)
    {
        float motorInput = _maxMotorTorque * vertical;
        float steeringAngle = _maxSteeringAngle * horizontal;

        throttleInput = motorInput;
        steeringInput = steeringAngle;

        foreach (AxleInfo axleInfo in _axleInfos)
        {
            if (axleInfo.steering)
            {
                axleInfo.leftWheel.steerAngle = steeringAngle;
                axleInfo.rightWheel.steerAngle = steeringAngle;
            }
            if (axleInfo.motor)
            {
                axleInfo.leftWheel.motorTorque = motorInput;
                axleInfo.rightWheel.motorTorque = motorInput;
            }

            UpdateWheelVisuals(axleInfo.leftWheel);
            UpdateWheelVisuals(axleInfo.rightWheel);
        }
    }
    public void UpdateWheelVisuals(WheelCollider collider)
    {
        if (collider.transform.childCount == 0)
        {
            return;
        }

        Transform visualWheel = collider.transform.GetChild(0);

        Vector3 position;
        Quaternion rotation;
        collider.GetWorldPose(out position, out rotation);

        visualWheel.transform.position = position;
        visualWheel.transform.rotation = rotation;
    }


    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.Tab))
        {
            EndEpisode();
        }

        speed = transform.InverseTransformDirection(rigidBody.velocity).z;

        if (speed < 0)
        {
            SetReward(-1f);
        }

    }


    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Boundary"))
        {
            SetReward(-10f);
            EndEpisode();
        }
    }


   

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.CompareTag("Reward"))
        {
            collectedRewards.Add(other.gameObject);
            other.gameObject.SetActive(false);
            SetReward(1f);
            rewardsCount++;
        }

        if ((rewardsCount % 5) == 0) 
        {
            ResetRewards();
        }
    }

  

   
   
}
