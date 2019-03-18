using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class EgoCarTrainingInterface : MonoBehaviour {

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

    // Car detection parameters
    private int numLineSegments = 100;
    private float detectionRadius = 7.5f;
    LineRenderer line;

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
        carState = new CarState(0, transform.position.z, transform.position.x, 0, length / 2, length / 2);
        goalPos = transform.position;
        SetTargetPosition(goalPos);

        //Draw the lidar visualization
        line = gameObject.GetComponent<LineRenderer>();
        line.SetVertexCount(numLineSegments + 1);
        line.useWorldSpace = false;
        DrawLidar();

        Debug.Log(string.Format("track width {0}", GetTrackWidth()));
        Debug.Log(string.Format("track length {0}", GetTrackLength()));
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
}
