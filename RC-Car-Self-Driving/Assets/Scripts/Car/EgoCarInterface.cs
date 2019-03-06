using System.Collections;
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
    private GameObject track;

    // Car info
    private Vector3 goalPos;
    private CarState carState;
    private Vector3 linearVel;
    private float angularVel = 0;

    // Car detection parameters
    private int numLineSegments = 100;
    private float detectionRadius = 7.5f;
    LineRenderer line;

    public float GetTrackWidth()
    {
        return track.gameObject.GetComponent<TrackSpeedUpdater>().GetComponent<MeshRenderer>().bounds.size.x;
    }

    public float GetTrackLength()
    {
        return track.gameObject.GetComponent<TrackSpeedUpdater>().GetComponent<MeshRenderer>().bounds.size.z;
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
        carController.goalPoseCallback(new Pose(new Vector3(position.z, position.x, 0), transform.rotation));
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
            obstaclePos.Add(collider.transform.position);
            Debug.Log(string.Format("obstacle position: {0}", collider.transform.position));
        }
        return obstaclePos;
    }

    private void Awake()
    {
        // Find track object so velocity can be fed to car controller
        track = GameObject.Find("MainTrack");
        if (track == null)
        {
            Debug.Log("The track object was not found.");
        }
    }

    private void Start()
    {
        InvokeRepeating("CalculateControls", 0f, 1 / Constants.targetHz);

        //Get dimensions of car sprite
        float length = GetComponentInChildren<Renderer>().bounds.size.x;
        float width = GetComponentInChildren<Renderer>().bounds.size.y;

        //Initialize the controllers and system
        carController = new CarController(length);
        steerSys = new FirstOrderSystem(Constants.carSimSteeringK, Constants.carSimSteeringTau, Constants.targetHz, 0f);
        throttleSys = new AsymmetricFirstOrderSystem(Constants.carSimVelK, Constants.carSimVelIncreaseTau, Constants.carSimVelDecreaseTau, Constants.targetHz, 0f);

        // Set initial car state
        carState = new CarState(0, transform.position.z, transform.position.x, 0, length / 2, length / 2);
        goalPos = transform.position;
        SetTargetPosition(goalPos);

        //Draw the lidar visualization
        line = gameObject.GetComponent<LineRenderer>();
        line.SetVertexCount(numLineSegments + 1);
        line.useWorldSpace = false;
        DrawLidar();
    }

    //Reset vehicle position on collision
    private void OnCollisionEnter(Collision collision)
    {
        Vector3 resetPosition = new Vector3(0, 1, 0);
        carState.x = resetPosition.x;
        carState.z = resetPosition.z;
        goalPos = resetPosition;
        SetTargetPosition(goalPos);
        Debug.Log(string.Format("Collision detected.  Resetting EgoCar position to {0}", resetPosition));
    }

    private void CalculateControls()
    {
        // Receive feedback of the current treadmill velocity
        carController.treadmillVelCallback(track);      // temporary

        //temporary
        Pose convertedCoordinatesPose = new Pose(new Vector3(transform.position.z, transform.position.x, 0), transform.rotation);
        Vector2 convertedCoordinatesLinearVel = new Vector2(linearVel.z, linearVel.x);
        CarController.CarCommand command = carController.syncCallback(convertedCoordinatesPose, convertedCoordinatesLinearVel, angularVel);

        //Update car controls from PID/Stanley Controllers
        steer = steerSys.Output(command.steer);
        throttle = throttleSys.Output(command.throttle);

        UpdateCar();
    }

    private void UpdateCar()
    {
        float dt = (1 / Constants.targetHz);
        float trackVel = track.gameObject.GetComponent<TrackSpeedUpdater>().vel;

        //Update kinematic bicycle model with new state
        carState.UpdateState(throttle, steer, dt, trackVel);

        //Update the state of the car sprite
        transform.position = new Vector3(carState.x, transform.position.y, carState.z);
        float newAngle = (carState.psi * Mathf.Rad2Deg);
        Quaternion rotation = Quaternion.Euler(0, (carState.psi * Mathf.Rad2Deg), 0);
        transform.rotation = rotation;

        //Update the sprite's velocity
        linearVel = new Vector3(carState.dx, 0, carState.dz);
        angularVel = carState.dv;       //is this right???
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
        SetTargetPosition(target);
        // AddReward(-0.1f);
        // resetCount = resetCount + 1;
        // if ((transform.position - GetTargetPosition()).sqrMagnitude < 5) {
        //     resetCount = 0;
        // }
        // Debug.Log(resetCount);
            // RequestDecision();
        // } else if (resetCount > 20) { // CHANGE THIS!!!
        //     Debug.Log("Running In Circles");
        //     resetCount = 0;
        //     RequestDecision();
        // }
        if (transform.position.x >= mapLength - 10) {
            Debug.Log("Finished");
            // resetCount = 0;
            SetReward(1.0f);
            finished = true;
            Done();
        } else if (
            GetTargetPosition().x < 10 || 
            GetTargetPosition().z < 10 ||
            GetTargetPosition().z > 40) {
            Debug.Log("Out Of Bounds");
            // resetCount = 0;
            SetReward(-1.0f);
            finished = false;
            Done();
        }
    }

    public override void AgentReset()
    {
        agentResetCount = agentResetCount + 1;
        Debug.Log(string.Format("Agent Reset: {0}", agentResetCount));
        Pathfinding.GetComponent<ObstaclesController>().InitObstacles();
        transform.position = new Vector3(startingXPos, 0, 25);
        transform.eulerAngles = new Vector3(0, 90, 0);

        //Reset target position
        targetPosition = transform.position;
        Debug.Log(transform.position);
        if (finished) {
            wins = wins + 1;
        } else if (transform.position.x > 30) {
            loses = loses + 1;
        }
        Debug.Log(wins);
        Debug.Log(loses);
    }

    private void OnCollisionEnter(Collision collision)
    {
        Debug.Log("Collision detected");
        SetReward(-1.0f);
        finished = false;
        Done();
    }

    //Search through all obstacles to find which fall within a radius of the car
    public List<ObstacleData> DetectNClosestObstacles(int N)
    {
        //The list with close obstacles
        List<ObstacleData> closeObstacles = new List<ObstacleData>();

        //The list with all obstacles in the map
        List<ObstacleData> allObstacles = ObstaclesController.obstaclesPosList;

        //Find close obstacles
        for (int i = 0; i < allObstacles.Count; i++)
        {
            closeObstacles.Add(allObstacles[i]);
        }
        List<ObstacleData> firstNClosestObstacles = closeObstacles
            .OrderBy(obs => (transform.position - obs.centerPos).magnitude)
            .Take(N)
            .ToList();

        return firstNClosestObstacles;
    }

    public override void CollectObservations() {
        Debug.Log("Collecting Observations");
        AddVectorObs(transform.position.x/mapLength);
        AddVectorObs(transform.position.z/mapWidth);

        int numOfObstaclesDetected = 5;
        List<ObstacleData> obstacleList = DetectNClosestObstacles(numOfObstaclesDetected);
        for (int i = 0; i < obstacleList.Count; i++) {
            AddVectorObs((obstacleList[i].centerPos.x - transform.position.x)/mapLength);
            AddVectorObs((obstacleList[i].centerPos.z - transform.position.z)/mapWidth);
            AddVectorObs((obstacleList[i].cornerPos.BL.x - transform.position.x)/mapLength);
            AddVectorObs((obstacleList[i].cornerPos.FR.x - transform.position.x)/mapLength);
            AddVectorObs((obstacleList[i].cornerPos.BL.z - transform.position.z)/mapWidth);
            AddVectorObs((obstacleList[i].cornerPos.FR.z - transform.position.z)/mapWidth);
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
            Debug.Log("Acting");
            float norm = Mathf.Sqrt(Mathf.Pow(vectorAction[0], 2) + Mathf.Pow(vectorAction[1], 2));
            target = new Vector3(
                Mathf.Clamp(transform.position.x + vectorAction[0]*20/norm, 0, mapLength), 
                0, 
                transform.position.z + vectorAction[1]*20/norm);
            // SetTargetPosition(waypoint);
            oldActionX = vectorAction[0];
            oldActionZ = vectorAction[1];
        }
        SetReward(transform.position.x - currentXPos);
        // Debug.Log(string.Format("Reward: {0}", transform.position.x - currentXPos));
        currentXPos = transform.position.x;
        // Debug.Log(transform.position);
        // Debug.Log(GetTargetPosition());
    }
}
