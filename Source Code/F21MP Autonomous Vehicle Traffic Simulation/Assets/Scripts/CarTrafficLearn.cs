using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using Assets.Scripts;

public class CarTrafficLearn : Agent
{
    [Header("Car Settings")]
    [SerializeField] private List<AxleInfo> _axleInfos;
    [SerializeField] private float _maxMotorTorque = 800;
    [SerializeField] private float _maxSteeringAngle = 30;
    [SerializeField] private Transform frontSensor;
    [SerializeField] private float m_BlockedDistance = 5f;

    private Rigidbody _rb;
    private Vector3 _initialPosition;
    private Quaternion _initialRotation;
    private bool _isInWaitZone = false;
    private WaitZone _currentWaitZone;
    private List<GameObject> _deactivatedRewards = new List<GameObject>();

    private void Awake()
    {
        _rb = GetComponent<Rigidbody>();
        _initialPosition = transform.position;
        _initialRotation = transform.rotation;
    }
    public Vector3 localVelocity;
    public bool m_Blocked;
    private void Update()
    {
        //Manually End Episode
        if (Input.GetKeyDown(KeyCode.R))
        {
            EndEpisode();
        }

        localVelocity = transform.InverseTransformDirection(_rb.velocity);

        //If its going backwards
        if (localVelocity.z < 0)
        {
            SetReward(-1f);
        }

        //If its colliding with the ground
        foreach (AxleInfo wheel in _axleInfos)
        {
            if (wheel.leftWheel.GetGroundHit(out WheelHit hit))
            {
                if (hit.collider.gameObject.CompareTag("Ground"))
                {
                    SetReward(-1f);
                }
            }

        }
    }
    private void FixedUpdate()
    {
        m_Blocked = CheckBlocked();

        // Check if the car is moving while it is blocked
        if (m_Blocked)
        {
            if (_rb.velocity.magnitude > 0.1f) // Check if the car is still moving, 0.1 can be considered a threshold to allow for some minor physics noise
            {
                _rb.velocity = Vector3.zero;  // Force the car to stop
                SetReward(-1f);  // Apply penalty for moving while blocked
            }
            else
            {
                SetReward(10f);  // Provide a positive reward for stopping due to an obstacle
            }
        }

        CheckTrafficLightCompliance();

    }

    private void CheckTrafficLightCompliance()
    {
        if (_isInWaitZone)
        {
            if (!_currentWaitZone.canPass && _rb.velocity.magnitude < 0.1f)
            {
                SetReward(3f);  // Reward for stopping at a red light
            }
            else if (!_currentWaitZone.canPass && _rb.velocity.magnitude > 0.1f)
            {
                SetReward(-1f);  // Reward for stopping at a red light
            }
            else if (_currentWaitZone.canPass && _rb.velocity.magnitude > 0.1f)
            {
                SetReward(3f);  // Reward for moving on green
            }
        }
    }

    public override void OnEpisodeBegin()
    {
        ResetCarPosition();
        ResetCheckpoints();
        _deactivatedRewards.Clear();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Vector3 localVelocity = transform.InverseTransformDirection(_rb.velocity);
        sensor.AddObservation(localVelocity.x);
        sensor.AddObservation(localVelocity.z);
        sensor.AddObservation(_rb.velocity.magnitude);
        sensor.AddObservation(CheckBlocked());
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        Move(actionBuffers.ContinuousActions[0], actionBuffers.ContinuousActions[1]);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Horizontal"); // Steering
        continuousActionsOut[1] = Input.GetAxis("Vertical");   // Acceleration/Braking
    }

    private void ResetCarPosition()
    {
        transform.position = _initialPosition;
        transform.rotation = _initialRotation;
        _rb.velocity = Vector3.zero;
        _rb.angularVelocity = Vector3.zero;
        StartCoroutine(PositionChangeWatchdog());
    }

    private void Move(float horizontal, float vertical)
    {
        float motor = _maxMotorTorque * vertical;
        float steering = _maxSteeringAngle * horizontal;

        foreach (AxleInfo axleInfo in _axleInfos)
        {
            if (axleInfo.steering)
            {
                axleInfo.leftWheel.steerAngle = steering;
                axleInfo.rightWheel.steerAngle = steering;
            }
            if (axleInfo.motor)
            {
                axleInfo.leftWheel.motorTorque = motor;
                axleInfo.rightWheel.motorTorque = motor;
            }
        }
    }

    private bool CheckBlocked()
    {
        RaycastHit hit;
        if (Physics.Raycast(frontSensor.position, transform.forward, out hit, m_BlockedDistance))
        {
            if (hit.collider.CompareTag("Gib") || hit.collider.CompareTag("Unit"))
            {
                AddReward(-0.5f);
                return true;
            }
        }
        return false;
    }

    private void OnTriggerEnter(Collider other)
    {
        HandleTriggerEnter(other);
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("WaitZone"))
        {
            _isInWaitZone = false;
            StopCoroutine(RewardForWaiting());
            _currentWaitZone = null;
        }
    }
    private void OnCollisionEnter(Collision collision)
    {
        switch (collision.gameObject.tag)
        {
            case "Boundary":
            case "Gib":
            case "Unit":
                SetReward(-1f);
                EndEpisode();
                break;
        }
    }
    private void HandleTriggerEnter(Collider other)
    {
        if (other.CompareTag("Reward"))
        {
            ProcessRewardItem(other);
        }
        else if (other.CompareTag("WaitZone"))
        {
            _isInWaitZone = true;
            _currentWaitZone = other.GetComponent<WaitZone>();
            if (_currentWaitZone)
            {
                StartCoroutine(RewardForWaiting());
            }
        }
    }
    public int _numRewardCollected;
    private void ProcessRewardItem(Collider other)
    {
        //SetReward(1.0f); // Positive reward for collecting rewards
        other.gameObject.SetActive(false);
        _deactivatedRewards.Add(other.gameObject);

        float dotProduct = Vector3.Dot(transform.forward, other.transform.forward);

        // Check if they are facing in the same direction
        if (dotProduct > 0.9) // Using 0.9 to allow for some small deviation
        {
            SetReward(-1f);
            Debug.Log("Negative reward: Facing same direction.");
        }
        // Check if they are facing each other (opposite directions)
        else if (dotProduct < -0.9) // Using -0.9 to allow for some small deviation
        {
            SetReward(5f);
            Debug.Log("Positive reward: Facing each other.");
        }
        else
        {
            // No reward or a different handling can be added here for perpendicular or other angles
            Debug.Log("No reward: Not facing same or opposite directions.");
        }
        _numRewardCollected++;
        if (_numRewardCollected % 5 == 0)
        {
            ResetCheckpoints();
        }
    }

    private void ResetCheckpoints()
    {
        foreach (GameObject reward in _deactivatedRewards)
        {
            reward.SetActive(true);
        }
    }

    private IEnumerator RewardForWaiting()
    {
        while (_isInWaitZone && !_currentWaitZone.canPass)
        {
            if (_rb.velocity.magnitude < 0.1f)
            {
                AddReward(0.1f);  // Reward for each second of waiting
                yield return new WaitForSeconds(1);
            }
            else
            {
                yield return null;
            }
        }
    }

    private IEnumerator PositionChangeWatchdog()
    {
        Vector3 positionBefore = transform.position;
        yield return new WaitForSeconds(120); // 2 minutes without significant position change
        if (Vector3.Distance(positionBefore, transform.position) < 0.5f)
        {
            SetReward(-1f); // Penalize for being stuck
            EndEpisode();
        }
        else
        {
            StartCoroutine(PositionChangeWatchdog());
        }
    }
}
