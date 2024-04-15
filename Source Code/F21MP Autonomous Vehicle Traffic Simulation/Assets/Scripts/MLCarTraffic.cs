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
public class MLCarTraffic : Agent
{
    [SerializeField]
    private List<AxleInfo> _axleInfos = null;
    [SerializeField]
    private float _maxMotorTorque = 800;
    [SerializeField]
    private float _maxSteeringAngle = 30;

    private Rigidbody _rb;
    private Vector3 _initialPosition;
    private Quaternion _initialRotation;
    private List<GameObject> _deactivatedRewards;
    public int _numRewardCollected;
    public Road _currentRoad;
    public NavConnection _currentOutConnection;

    public override void Initialize()
    {
        _rb = GetComponent<Rigidbody>();
        _deactivatedRewards = new List<GameObject>();
        _initialPosition = transform.position;
        _initialRotation = transform.rotation;
    }

    public int roadConnectionIndex;
    public override void OnEpisodeBegin()
    {
        _rb.velocity = Vector3.zero;
        transform.position = _initialPosition;
        transform.rotation = _initialRotation;
        angleToConnection = 0;
        localVelocity = 0;
        normalizedDistanceToObstacle = -1.0f;
        obstacleDetected = false;
        ResetCheckpoints();
        _deactivatedRewards.Clear();
        // Reset road logic
        _currentRoad = FindStartingRoad();
        _currentOutConnection = _currentRoad?.GetOutConnectionRoad(roadConnectionIndex);
    }

    public float localVelocity;

    public float detectionRange = 5f;
    public float rayAngleSpread = 45f;
    public int numRays = 5;
    private void Update()
    {
        //Manually End Episode
        if (Input.GetKeyDown(KeyCode.R))
        {
            EndEpisode();
        }

        localVelocity = transform.InverseTransformDirection(_rb.velocity).z;

        //If its going backwards
        if (localVelocity < -0.1f)
        {
            SetReward(-0.5f);
        }


        //Detect Objects Train

        RaycastHit hit;

        Vector3 forward = transform.forward;
        Quaternion startingAngle = Quaternion.AngleAxis(-rayAngleSpread / 2, transform.up);
        Vector3 rayDirection = startingAngle * forward;
        float angleIncrement = rayAngleSpread / numRays;

        for (int i = 0; i <= numRays; i++)
        {
            Debug.DrawRay(transform.position, rayDirection * detectionRange, Color.red, 0.1f);

            if (Physics.Raycast(transform.position, rayDirection, out hit, detectionRange))
            {
                // Check if the ray hit a pedestrian or car
                if (hit.collider.CompareTag("Gib") || hit.collider.CompareTag("Unit"))
                {
                    Debug.Log("Obstacle detected: " + hit.collider.tag);
                   // HandleObstacleDetected(hit.distance);
                    Debug.DrawRay(transform.position, rayDirection * hit.distance, Color.green, 1f);
                    obstacleDetected = true;
                    normalizedDistanceToObstacle = hit.distance / detectionRange;

                    // Updated reward calculation based on urgency
                    float rewardValue = Mathf.Clamp(0.5f * (1f - normalizedDistanceToObstacle), 0.1f, 0.5f); // More reward for closer detections
                    SetReward(rewardValue);

                    _rb.velocity = Vector3.zero;// Stage3_PPO1

                    Debug.Log($"Stopping due to obstacle, reward: {rewardValue}");

                    break; // Stop checking other rays once an obstacle is detected
                }
            }

            // Rotate the ray direction
            rayDirection = Quaternion.AngleAxis(angleIncrement, transform.up) * rayDirection;
        }

        // Detect Objects Train


    }
    private float normalizedDistanceToObstacle = -1.0f;  // Initialize to -1 to indicate no obstacle detected
    private bool obstacleDetected = false;  // Flag to indicate obstacle presence


    // Detect Objects Train
    void HandleObstacleDetected(float distance)
    {
        // Simple reaction: stop the car
        _rb.velocity = Vector3.zero;

        // Issue a reward or penalty based on the distance the obstacle was detected
        float rewardValue = Mathf.Clamp(1f - (distance / 50.0f), 0.1f, 1f); // More reward for detecting obstacles from further away
        SetReward(rewardValue);
        Debug.Log("Stopping due to obstacle, reward: " + rewardValue);

        // Optionally, end the episode if that fits your training scenario
        //EndEpisode();


    }
    // Detect Objects Train
    public override void CollectObservations(VectorSensor sensor)
    { 
        localVelocity = transform.InverseTransformDirection(_rb.velocity).z;
        sensor.AddObservation(localVelocity);
        sensor.AddObservation(localVelocity);
        sensor.AddObservation(_rb.velocity.magnitude);
        sensor.AddObservation(transform.localEulerAngles.y);
        if(_currentOutConnection)
            sensor.AddObservation(_currentOutConnection.transform.localEulerAngles.y);

        //Added For Training to Curve

        if (_currentRoad != null)
        {
            if (_currentRoad.isCurve)
            {
                sensor.AddObservation(angleToConnection);
            }
        }
        //Added For Training to Curve


        sensor.AddObservation(normalizedDistanceToObstacle);

    }

    //Added For Training to Curve


    [SerializeField]
    private float _curveSteeringGain = 1.0f; // Adjust based on desired sensitivity to curve detection

    [SerializeField]
    private float _maxSteeringAdjustment = 30.0f; // Maximum allowable steering adjustment in degrees

    //Added For Training to Curve

    float safeStoppingDistanceNormalized = 0.25f;
    float safeSpeed = 5f;
    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        float horizontal = actionBuffers.ContinuousActions[0];
        float vertical = actionBuffers.ContinuousActions[1];

        /*
        //Added For Training to Curve


        // Calculate desired steering angle based on action input
        float steering = horizontal * _maxSteeringAngle;

        // Apply motor torque for acceleration/braking
        float motor = vertical * _maxMotorTorque;

        // Apply rotation adjustment for upcoming curves (optional)
        if (_currentRoad != null && _currentRoad.isCurve)
        {
            // Calculate desired alignment angle for curves based on stored angleToConnection
            float targetAngle = angleToConnection;

            // Calculate adjustment needed to align with the target angle (e.g., for curves)
            float currentAngle = transform.eulerAngles.y;
            float angleDifference = Mathf.DeltaAngle(currentAngle, targetAngle);

            // Apply a steering adjustment based on the angle difference (e.g., proportional control)
            float steeringAdjustment = Mathf.Clamp(angleDifference * _curveSteeringGain, -_maxSteeringAdjustment, _maxSteeringAdjustment);

            // Update the steering angle with the adjustment for curves
            steering += steeringAdjustment;
        }

        // Move the car based on calculated steering and motor inputs
        Move(steering, motor);

        //Added For Training to Curve

        */


        // Check if the car is on a curve and needs to adjust its steering
        if (_currentRoad != null && _currentRoad.isCurve && _currentOutConnection != null)
        {
            // Calculate desired alignment angle for curves based on stored angleToConnection
            float currentAngle = transform.localEulerAngles.y;
            float targetAngle = Mathf.DeltaAngle(currentAngle, angleToConnection);

            // Adjust the steering to align with the target angle
            // This might involve some form of proportional control or a direct setting based on the angle
            float steeringAdjustment = -Mathf.Clamp(targetAngle * _curveSteeringGain, -_maxSteeringAdjustment, _maxSteeringAdjustment);
            Move(horizontal, vertical, steeringAdjustment);

            // Optionally, reset angleToConnection to zero once aligned
            if (Mathf.Abs(targetAngle) < 5.0f) // Threshold for 'alignment'
            {
                angleToConnection = 0;
            }
        }
        else
        {
            Move(horizontal, vertical);
        }

        //Move(horizontal, vertical);

        if (obstacleDetected)
        {
            if (normalizedDistanceToObstacle < safeStoppingDistanceNormalized && _rb.velocity.magnitude < safeSpeed)
            {
                SetReward(1.0f);  // Reward for stopping safely
            }
            else
            {
                SetReward(-0.1f * normalizedDistanceToObstacle);  // Penalize based on distance to the obstacle when stopped
            }
        }
    }
    

    // Force Car to turn

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
    private void Move(float horizontal, float vertical, float? overrideSteeringAngle = null)
    {
        //Detect Object Train
        float currentSpeed = Vector3.Dot(_rb.velocity, transform.forward);
        float brakeStrength = Mathf.Clamp01(-vertical); // Assuming negative throttle values are for braking
        if (vertical < 0)
        {
            _rb.AddForce(-transform.forward * brakeStrength * 1500); // Apply a braking force
        }
        else
        {
            _rb.AddForce(transform.forward * vertical * 1000); // Apply driving force
        }

        ////Detect Object Train

        float motor = _maxMotorTorque * vertical;
       // float steering = _maxSteeringAngle * horizontal;

        float steering = overrideSteeringAngle ?? (_maxSteeringAngle * horizontal);

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

            ApplyLocalPositionToVisuals(axleInfo.leftWheel);
            ApplyLocalPositionToVisuals(axleInfo.rightWheel);
        }
    }

    // Method to calculate deviation from lane center
    private float CalculateDeviationFromLaneCenter(Vector3 position, Vector3 laneCenterPosition)
    {
        return Vector3.Distance(position, laneCenterPosition);
    }

    // Calculate reward based on deviation
    private float CalculateReward(float deviation)
    {
        // Use an exponential decay function to calculate reward based on deviation
        // The smaller the deviation, the higher the reward
        return Mathf.Exp(-deviation);
    }

    public string laneName;
    private void OnTriggerEnter(Collider other)
    {

        //Detect Objects Train

        if (other.CompareTag("Unit")) // Assuming "Unit" is the tag for pedestrians
        {
            Debug.Log("Pedestrian detected! Stopping car.");
           // _rb.velocity = Vector3.zero; // Stop the car immediately
            SetReward(-1f); // Penalize the agent for close proximity without stopping sooner

            normalizedDistanceToObstacle = -1.0f;
            obstacleDetected = false;
            //EndEpisode(); // Optionally end the episode if the goal is to avoid hitting pedestrians 
        }

        //Detect Objects Train


        if (other.CompareTag("Reward"))
        {
            LaneReward rewardLaneName = other.GetComponent<LaneReward>();
            if (rewardLaneName != null)
            {
                if (rewardLaneName.laneName == laneName)
                {
                    // Correct lane
                    //SetReward(1f);  // Adjust reward magnitude as needed
                    //Debug.Log("Correct lane! Positive reward.");

                    // Calculate deviation from the lane center
                    float deviation = CalculateDeviationFromLaneCenter(transform.position, rewardLaneName.transform.position);

                    // Calculate reward based on deviation
                    float reward = CalculateReward(deviation) *10f;

                    SetReward(reward);
                    Debug.Log($"Correct lane! Reward: {reward} based on deviation: {deviation}");
                }
                else
                {
                    // Wrong lane
                    SetReward(-2f);  // Adjust penalty magnitude as needed
                    Debug.Log("Wrong lane! Negative reward.");
                }
            }

            _currentRoad = FindStartingRoad();


            //Added For Training to Curve

            if (_currentRoad.isCurve)
            {
                if (_currentOutConnection)
                {
                    Vector3 directionToConnection = _currentOutConnection.transform.position - transform.position;
                    angleToConnection = Vector3.SignedAngle(transform.forward, directionToConnection, Vector3.up);
                }
            }
            //Added For Training to Curve


            _deactivatedRewards.Add(other.gameObject);
            other.gameObject.SetActive(false);
        }

        if (other.CompareTag("RoadConnection"))
        {
            NavConnection connection = other.GetComponent<NavConnection>();
            if(connection != null) 
            {
                if (connection.outConnections.Length > 0)
                {
                    _currentOutConnection=connection.GetOutConnection();

                    
                    //Added For Training to Curve
                    
                    Vector3 directionToNextConnection = (connection.outConnections[0].transform.position - transform.position).normalized;
                    float angleToNextConnection = Vector3.SignedAngle(transform.forward, directionToNextConnection, Vector3.up);

                    // Reward for aligning with the next road connection (turning correctly)
                    float maxAllowedAngle = 30f; // Adjust as needed based on the desired turning accuracy
                    if (Mathf.Abs(angleToNextConnection) < maxAllowedAngle)
                    {
                        SetReward(1f);
                        Debug.Log("Correct turn! Positive reward."+ angleToConnection);
                    }
                    else
                    {
                        SetReward(-1f);
                        Debug.Log("Incorrect turn! Negative reward.");
                    }


                    //Added For Training to Curve

                }
                else 
                {
                    if (connection == _currentOutConnection) 
                    {
                        Debug.LogError("Hit correct connection" + _currentOutConnection);

                        _currentOutConnection = null;

                        SetReward(2f);
                    }
                    else 
                    {
                        Debug.LogError("Hit Wrong connection" + _currentOutConnection);
                        _currentOutConnection = null;
                        SetReward(-2f);
                    }
                }
            }
            
          
        }


        // Traffic Signal Learn

        if (other.CompareTag("WaitZone"))
        {
            WaitZone waitZone = other.GetComponent<WaitZone>();
            if (waitZone != null)
            {
                if (!waitZone.canPass)
                {
                    _rb.velocity = Vector3.zero; //Stage3_PPO1
                    SetReward(-1f); // Penalize for entering a non-passable waitzone
                    Debug.Log("Entered non-passable WaitZone! Negative reward.");
                    // Optional: Add functionality to stop the car
                }
                else
                {
                    SetReward(1f); // Reward for correctly navigating through a passable waitzone
                    Debug.Log("Entered passable WaitZone! Positive reward.");
                }
            }
        }

        // Traffic Signal Learn
    }



    private void OnTriggerStay(Collider other)
    {
        if (other.CompareTag("WaitZone"))
        {
            WaitZone waitZone = other.GetComponent<WaitZone>();
            if (waitZone != null && !waitZone.canPass)
            {
                if (!waitZone.canPass) 
                {
                    if (_rb.velocity.magnitude > 0.1) // Check if the car is still moving
                    {
                         _rb.velocity = Vector3.zero; // Stage3_PPO1
                        SetReward(-0.1f); // Continuous penalty for moving when should be stopped
                        Debug.Log("Moving in non-passable WaitZone! Continuous negative reward.");
                    }
                    else
                    {
                        SetReward(0.1f); // Continuous reward for stopping as required
                        Debug.Log("Stopped in non-passable WaitZone! Continuous positive reward.");
                    }
                }
                else 
                {
                    // Minor penalty for staying unnecessarily in a passable zone
                    SetReward(-0.05f); // Encourage the agent to keep moving
                    Debug.Log("Unnecessarily staying in a passable WaitZone! Minor negative reward.");
                }
               
            }
        }
    }
    private void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("WaitZone"))
        {
            WaitZone waitZone = other.GetComponent<WaitZone>();
            if (waitZone != null && !waitZone.canPass)
            {
                Debug.Log("Exiting non-passable WaitZone.");
                // Reset or adjust car state ready for normal driving
                SetReward(0.1f); // Reward for correctly leaving the WaitZone and resuming normal driving
            }
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Boundary") || collision.gameObject.CompareTag("Gib"))
        {
            // Calculate the impact speed, which is a factor in determining the penalty
            float impactSpeed = _rb.velocity.magnitude;

            // Determine the penalty based on the impact speed
            float penalty = Mathf.Clamp(-0.5f * impactSpeed, -1f, -0.1f); // Ensures penalty is between -0.1 and -1
            SetReward(penalty);
            Debug.Log($"Collision at speed {impactSpeed} with penalty {penalty}.");

            normalizedDistanceToObstacle = -1.0f;
            obstacleDetected = false;
            EndEpisode();
        }
    }

    private void ResetCheckpoints()
    {
        foreach (GameObject reward in _deactivatedRewards)
        {
            reward.SetActive(true);
        }
    }

    public Transform DownwardVisionX;
    public float angleToConnection;
    private Road FindStartingRoad()
    {
        
        Transform rayOrigin = DownwardVisionX; // Name of the child transform
        if (rayOrigin == null)
        {
            Debug.LogError("DownwardVisionX transform not found on the car object.");
            return null;
        }

        RaycastHit hit;
        float rayLength = 10f; // Set this based on how high the car is off the ground

        // Cast a ray straight down from the rayOrigin
        if (Physics.Raycast(rayOrigin.position, -rayOrigin.up, out hit, rayLength))
        {
            // Check if the hit collider has a Road component in its parents
            Road road = FindRoadInParents(hit.collider.transform);
            if (road != null)
            {
                Debug.LogError("Detected Road"+ road);
                return road;
            }
            else
            {
                Debug.Log("Road component not found on the hit object's parents.");
            }
        }
        else
        {
            Debug.Log("No ground detected below the car.");
        }

        return null;
    }

    private Road FindRoadInParents(Transform child)
    {
        while (child != null)
        {
            Road road = child.GetComponent<Road>();
            if (road != null)
                return road;
            child = child.parent;
        }
        return null;
    }
    public void ApplyLocalPositionToVisuals(WheelCollider collider)
    {
        if (collider.transform.childCount == 0)
            return;

        Transform visualWheel = collider.transform.GetChild(0);
        Vector3 position;
        Quaternion rotation;
        collider.GetWorldPose(out position, out rotation);

        visualWheel.transform.position = position;
        visualWheel.transform.rotation = rotation;
    }

    private IEnumerator PositionChangeWathdog()
    {
        Vector3 positionBefore = transform.position;

        yield return new WaitForSeconds(60f);

        float distance = Vector3.Distance(positionBefore, transform.position);
        if (distance < 0.5f)
        {

            normalizedDistanceToObstacle = -1.0f;
            obstacleDetected = false;

            EndEpisode();
        }
        else
        {
            StartCoroutine(PositionChangeWathdog());
        }

    }
}
