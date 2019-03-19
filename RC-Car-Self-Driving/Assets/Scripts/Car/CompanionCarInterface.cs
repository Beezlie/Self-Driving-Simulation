using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//references https://bitbucket.org/dshin-uwaterloo/adas-car-on-treadmill-v2/src/master/src/vrep_sim/src/sim_car.cpp
public class CompanionCarInterface : MonoBehaviour
{
    // Car Controls
    private CarController carController;
    private FirstOrderSystem steerSys;
    private AsymmetricFirstOrderSystem throttleSys;
    private float steer;        // in radians       
    private float throttle;     // in percentage
    private GameObject track;
    private GameObject road;

    // Car info
    private Vector3 goalPos;
    private CarState carState;
    private Vector3 linearVel = new Vector3(0, 0, 0);
    private float angularVel = 0;
    private float acceleration = 2f;

    public void SetTargetPosition(Vector3 position)
    {
        carController.setNewGoalPosition(new Pose(new Vector3(position.z, position.x, 0), transform.rotation));
        goalPos = position;
        // Debug.Log(string.Format("Companion car target position set: {0}.", goalPos));
    }

    public Vector3 GetTargetPosition()
    {
        return goalPos;
    }

    private void Awake()
    {
        // Find track object so velocity can be fed to car controller
        track = GameObject.Find("TrackData");
        if (track == null)
        {
            Debug.Log("The track object was not found.");
        }
        road = GameObject.Find("Road Piece");
        if (road == null)
        {
            Debug.Log("The Road Piece object was not found.");
        }
    }

    private void Initialize()
    {
        //Get dimensions of car sprite
        float length = GetComponentInChildren<Renderer>().bounds.size.x;
        float width = GetComponentInChildren<Renderer>().bounds.size.y;

        carController = new CarController(length);
        steerSys = new FirstOrderSystem(Constants.carSimSteeringK, Constants.carSimSteeringTau, Constants.targetHz, 0f);
        throttleSys = new AsymmetricFirstOrderSystem(Constants.carSimVelK, Constants.carSimVelIncreaseTau, Constants.carSimVelDecreaseTau, Constants.targetHz, 0f);

        // Set initial car state
        carState = new CarState(0, Random.Range(5, road.gameObject.GetComponent<MeshRenderer>().bounds.size.z-5), Random.Range(5, road.gameObject.GetComponent<MeshRenderer>().bounds.size.x-5), 0, length / 2, length / 2, acceleration);
    }

    private void Start()
    {
        InvokeRepeating("UpdateCar", 0f, 1 / Constants.targetHz);
        Initialize();
    }

    void UpdateCar()
    {
        //Get updated track velocity
        float dt = (1 / Constants.targetHz);
        float trackVel = track.gameObject.GetComponent<TrackSpeedSubscriber>().GetVelocity();

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
        if (carState.x < 0 || 
            carState.x > road.gameObject.GetComponent<MeshRenderer>().bounds.size.x || 
            carState.z < 0 ||
            carState.z > road.gameObject.GetComponent<MeshRenderer>().bounds.size.z) {
            Debug.Log("Out Of Bounds");
            Initialize();
        }
    }
}


