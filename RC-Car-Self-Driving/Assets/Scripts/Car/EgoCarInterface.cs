﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MLAgents;
using System.Linq;

public class EgoCarInterface : Agent {

    // Car Controls
    private CarController carController;
    private FirstOrderSystem steerSys;
    private AsymmetricFirstOrderSystem throttleSys;
    private float steer;        // in radians       
    private float throttle;     // in percentage
    private GameObject trackData;
    private GameObject road;
    private GameObject companionCarManager;

    // Car info
    private Vector3 goalPos;
    private CarState carState;
    private Vector3 linearVel;
    private float angularVel = 0;
    private float acceleration = 10f;

    // Car detection parameters
    private int numLineSegments = 100;
    private float detectionRadius = 7.5f;
    LineRenderer line;

    private float oldActionX = 0;
    private float oldActionZ = 0;
    private int agentResetCount = 0;
    public int numOfObstaclesDetected = 2;
    private List<GameObject> companionCars = new List<GameObject>();
    private List<GameObject> pursuingCars;

    public float GetTrackWidth()
    {
        return road.gameObject.GetComponent<MeshRenderer>().bounds.size.x;
    }

    public float GetTrackLength()
    {
        return road.gameObject.GetComponent<MeshRenderer>().bounds.size.z;
    }

    public Transform GetCurrentTransform()
    {
        return transform;
    }

    public Vector3 GetCurrentPosition()
    {
        return transform.position;
    }

    public float GetCurrentHeading()
    {
        return carState.psi;
    }

    private float GetCurrentSpeed()
    {
        return carState.dz;
    }

    public void SetTargetPosition(Vector3 position)
    {
        carController.setNewGoalPosition(new Pose(new Vector3(position.z, position.x, 0), transform.rotation));
        goalPos = position;
    }

    public Vector3 GetTargetPosition()
    {
        return goalPos;
    }

    //Search through all obstacles to find which fall within a detection radius of the car
    public List<Vector3> DetectObstaclesWithinRadiusOfCar()
    {
        List<Vector3> obstaclePos = new List<Vector3>();
        Collider[] obstacles = Physics.OverlapSphere(transform.position, detectionRadius);
        foreach (Collider collider in obstacles)
        {
            if (collider is BoxCollider && collider.transform.position != transform.position)
            {
                obstaclePos.Add(collider.transform.position);
                Debug.Log(string.Format("obstacle position: {0}", collider.transform.position));
            }
        }
        return obstaclePos;
    }

    //Return N closest cars
    public List<GameObject> DetectNClosestObstacles(int N)
    {
        companionCars = companionCars
            .OrderBy(car => (transform.position - car.transform.position).magnitude)
            .Take(N)
            .ToList();

        return companionCars;
    }

    public void SetCompanionCarMode(string companionCarName, CompanionCarManager.DrivingMode mode)
    {
        companionCarManager.gameObject.GetComponent<CompanionCarManager>().SetDrivingMode(companionCarName, mode);
    }

    private void Awake()
    {
        // Find track object so velocity can be fed to car controller
        trackData = GameObject.Find("TrackData");
        if (trackData == null)
        {
            Debug.Log("The trackData object was not found.");
        }
        road = GameObject.Find("Road Piece");
        if (road == null)
        {
            Debug.Log("The Road Piece object was not found.");
        }
        companionCarManager = GameObject.Find("CompanionCars");
        if (companionCarManager == null)
        {
            Debug.Log("The CompanionCars object was not found.");
        }
    }

    private void Start()
    {
        InvokeRepeating("UpdateCar", 0f, 1 / Constants.targetHz);

        //Get dimensions of car sprite
        float length = GetComponentInChildren<Renderer>().bounds.size.x;
        float width = GetComponentInChildren<Renderer>().bounds.size.y;

        //Initialize the controllers and system
        carController = new CarController(length);
        steerSys = new FirstOrderSystem(Constants.carSimSteeringK, Constants.carSimSteeringTau, Constants.targetHz, 0f);
        throttleSys = new AsymmetricFirstOrderSystem(Constants.carSimVelK, Constants.carSimVelIncreaseTau, Constants.carSimVelDecreaseTau, Constants.targetHz, 0f);

        // Set initial car state
        carState = new CarState(0, transform.position.z, transform.position.x, 0, length / 2, length / 2, acceleration);
        goalPos = transform.position;
        SetTargetPosition(goalPos);

        //Draw the lidar visualization
        line = gameObject.GetComponent<LineRenderer>();
        line.SetVertexCount(numLineSegments + 1);
        line.useWorldSpace = false;
        DrawLidar();

        Debug.Log(string.Format("track width {0}", GetTrackWidth()));
        Debug.Log(string.Format("track length {0}", GetTrackLength()));
        companionCars = companionCarManager.gameObject.GetComponent<CompanionCarManager>().GetCompanionCars();
        Debug.Log(string.Format("pursuingCars: {0}", pursuingCars));
        pursuingCars = companionCarManager.gameObject.GetComponent<CompanionCarManager>().GetPursuingCars();
        Debug.Log(string.Format("pursuingCars: {0}", pursuingCars));
    }

    //Reset vehicle position on collision
    private void OnCollisionEnter(Collision collision)
    {
        Debug.Log("Collision detected");
        SetReward(-1.0f);
        Done();
    }

    void UpdateCar()
    {
        //Get updated track velocity
        float dt = (1 / Constants.targetHz);
        float trackVel = trackData.gameObject.GetComponent<TrackSpeedSubscriber>().GetVelocity();

        //Calculate car controls
        Pose convertedCoordinatesPose = new Pose(new Vector3(transform.position.z, transform.position.x, 0), transform.rotation);
        Vector2 convertedCoordinatesLinearVel = new Vector2(linearVel.z, linearVel.x);
        CarController.CarCommand command = carController.calculateControls(convertedCoordinatesPose, convertedCoordinatesLinearVel, angularVel, trackVel);
        steer = steerSys.Output(command.steer);
        throttle = throttleSys.Output(command.throttle);

        //Update bicycle model
        carState.UpdateState(throttle, steer, dt, trackVel);

        //Update the state of the car sprite
        transform.position = new Vector3(carState.x, transform.position.y, carState.z);
        float newAngle = (carState.psi * Mathf.Rad2Deg);
        Quaternion rotation = Quaternion.Euler(0, (carState.psi * Mathf.Rad2Deg), 0);
        transform.rotation = rotation;
        linearVel = new Vector3(carState.dx, 0, carState.dz);
        angularVel = carState.dv;
    }

    // Draw circle around car representing detection range of Lidar sensor
    private void DrawLidar()
    {
        float x;
        float y;
        float z;

        float angle = 20f;

        for (int i = 0; i < (numLineSegments + 1); i++)
        {
            x = Mathf.Sin(Mathf.Deg2Rad * angle) * detectionRadius;
            z = Mathf.Cos(Mathf.Deg2Rad * angle) * detectionRadius;

            line.SetPosition(i, new Vector3(x, 0, z));

            angle += (360f / numLineSegments);
        }
    }

//////////////////////////////////////////////////

    // Update is called once per frame
    void Update()
    {
        AddReward(0.1f);
        if (GetCurrentPosition().x < 0 || 
            GetCurrentPosition().x > GetTrackWidth() || 
            GetCurrentPosition().z < 0 ||
            GetCurrentPosition().z > GetTrackLength()) {
            Debug.Log("Out Of Bounds");
            SetReward(-1.0f);
            Done();
        }
        foreach (GameObject car in companionCars) {
            if ((transform.position - car.transform.position).magnitude < 2.5) {
                Debug.Log("Collision detected");
                SetReward(-1.0f);
                Done();
            }
            // Debug.Log(string.Format("Reward: {0}", ((transform.position - pursuingCar.transform.position).magnitude)/10));
        }
    }

    public override void AgentReset()
    {
        agentResetCount = agentResetCount + 1;
        Debug.Log(string.Format("Agent Reset: {0}", agentResetCount));

        //Reset target position
        Vector3 resetPosition = new Vector3(Random.Range(5, GetTrackWidth()-5), 0, Random.Range(5, GetTrackLength()-5));
        carState.x = resetPosition.x;
        carState.z = resetPosition.z;
        carState.v = 0;
        carState.psi = 0;
        goalPos = resetPosition;
        SetTargetPosition(goalPos);
        Debug.Log(string.Format("Resetting EgoCar position to {0}", resetPosition));
    }

    public override void CollectObservations() {
        // Debug.Log("Collecting Observations");
        AddVectorObs(transform.position.x/GetTrackWidth());
        AddVectorObs(transform.position.z/GetTrackLength());

        List<GameObject> obstacleList = DetectNClosestObstacles(numOfObstaclesDetected);
        for (int i = 0; i < obstacleList.Count; i++) {
            AddVectorObs(obstacleList[i].transform.position.x/GetTrackWidth());
            AddVectorObs(obstacleList[i].transform.position.z/GetTrackLength());
        }
        if (numOfObstaclesDetected > obstacleList.Count) {
            for (int i = 0; i < numOfObstaclesDetected - obstacleList.Count; i++) {
                AddVectorObs(0);
                AddVectorObs(0);
            }
        }
    }

    public override void AgentAction(float[] vectorAction, string textAction) {
        if (vectorAction[0] != oldActionX || vectorAction[1] != oldActionZ) {
            // Debug.Log("Acting");
            // float norm = Mathf.Sqrt(Mathf.Pow(vectorAction[0], 2) + Mathf.Pow(vectorAction[1], 2));
            SetTargetPosition(new Vector3(
                Mathf.Clamp(transform.position.x + GetTrackWidth()*vectorAction[0]/8, 5, GetTrackWidth()-5), 
                0, 
                Mathf.Clamp(transform.position.z + GetTrackLength()*vectorAction[1]/4, 5, GetTrackLength()-5)));
            oldActionX = vectorAction[0];
            oldActionZ = vectorAction[1];
            // Debug.Log(string.Format("Target Position: {0}", GetTargetPosition()));
        }
    }
}
