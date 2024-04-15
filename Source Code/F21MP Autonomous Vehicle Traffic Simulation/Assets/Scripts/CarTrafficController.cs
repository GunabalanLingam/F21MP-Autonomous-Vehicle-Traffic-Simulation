using Assets.Scripts;
using JetBrains.Annotations;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Runtime.CompilerServices;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using UnityEngine.SocialPlatforms;
using UnityEngine.UIElements;

public class CarTrafficController : Agent
{
    [Header("Car Settings")]
    [SerializeField] private List<AxleInfo> _axleInfos = null;
    [SerializeField] private float _maxMotorTorque = 800;
    [SerializeField] private float _maxSteeringAngle = 30;

    private Rigidbody _rb;
    private Vector3 _initialPosition;
    private Quaternion _initialRotation;
    public List<GameObject> _deactivatedRewards;
    public int _numRewardCollected;

    public bool _isInWaitZone = false;
    public WaitZone _currentWaitZone;

    private void Start()
    {
        _initialPosition = transform.position;
        _initialRotation = transform.rotation;
    }

    public override void Initialize()
    {
        _rb = GetComponent<Rigidbody>();
        _deactivatedRewards = new List<GameObject>();
        _initialPosition = transform.position;
        _initialRotation = transform.rotation;
    }

    [SerializeField] private Transform front;  // Assuming this is the front of the car
    [SerializeField] private float m_BlockedDistance = 5f;  // Distance to consider "blocked"
    private bool m_Blocked;

    private bool CheckBlocked()
    {
        Vector3 forward = transform.TransformDirection(Vector3.forward);
        RaycastHit hit;
        if (Physics.Raycast(front.position, forward, out hit, m_BlockedDistance))
        {
            if (hit.transform.CompareTag("Gib") || hit.transform.CompareTag("Unit"))
                return true;
        }
        return false;
    }

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
        StartCoroutine(PositionChangeWatchdog());
        transform.position = _initialPosition;
        transform.rotation = _initialRotation;
        _rb.velocity = Vector3.zero;
        ResetCheckpoints();
        _deactivatedRewards.Clear();
    }
    Vector3 localVelocity;
    public override void CollectObservations(VectorSensor sensor)
    {
        localVelocity = transform.InverseTransformDirection(_rb.velocity);
        sensor.AddObservation(localVelocity.x);
        sensor.AddObservation(localVelocity.z);
        sensor.AddObservation(_rb.velocity.magnitude);
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        Move(actionBuffers.ContinuousActions[0], actionBuffers.ContinuousActions[1]);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continuousActionsOut = actionsOut.ContinuousActions;

        // Reset the continuous action values
        continuousActionsOut[0] = 0f;
        continuousActionsOut[1] = 0f;

        // Set the continuous actions based on user input
        continuousActionsOut[0] = Input.GetAxis("Horizontal"); // Steering
        continuousActionsOut[1] = Input.GetAxis("Vertical");   // Acceleration/Braking

    }

    private void OnTriggerEnter(Collider other)
    {
        switch (other.tag)
        {
            case "Reward":
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

                // Assuming you might still want to deactivate the reward object
                other.gameObject.SetActive(false);
                _numRewardCollected++;
                if ((_numRewardCollected % 5) == 0)
                {
                    ResetCheckpoints();
                }
                break;
            case "WaitZone":
                _isInWaitZone = true;
                _currentWaitZone = other.GetComponent<WaitZone>(); 
                if (_currentWaitZone && !_currentWaitZone.canPass)
                {
                    StartCoroutine(RewardForWaiting());
                    //SetReward(1f); // Positive reward for correctly waiting at a red light
                }
                break;
        }
    }

  
    public void ApplyLocalPositionToVisuals(WheelCollider collider)
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
    private void ResetCheckpoints()
    {
        foreach (GameObject reward in _deactivatedRewards)
        {
            reward.SetActive(true);
        }
    }
    private void OnTriggerExit(Collider other)
    {
        if (other.tag == "WaitZone")
        {
            _isInWaitZone = false;
            if (_currentWaitZone && !_currentWaitZone.canPass)
            {
               
                SetReward(-1f); // Negative reward for leaving the waitzone when not allowed
            }
            StopCoroutine(RewardForWaiting());
            _currentWaitZone = null;
        }
    }

    private IEnumerator RewardForWaiting()
    {
        while (_isInWaitZone && !_currentWaitZone.canPass)
        {
            if (_rb.velocity.magnitude < 0.1f)
            {
                SetReward(0.5f);  // Reward for each second of waiting
                yield return new WaitForSeconds(1);
            }
            else
            {
                yield return null;
            }
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

    private IEnumerator PositionChangeWatchdog()
    {
        Vector3 positionBefore = transform.position;
        yield return new WaitForSeconds(120); // 2 minutes without significant position change
        if (Vector3.Distance(positionBefore, transform.position) < 0.5f)
        {
            SetReward(-10f); // Penalize for being stuck
            EndEpisode();
        }
        else
        {
            StartCoroutine(PositionChangeWatchdog());
        }
    }

    // Additional methods to apply visual transformations, handle checkpoints, etc.
}
