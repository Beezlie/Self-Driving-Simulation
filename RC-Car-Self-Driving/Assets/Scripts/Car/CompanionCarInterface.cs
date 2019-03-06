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

    // Car info
    private CarState carState;
    private Vector3 linearVel = new Vector3(0, 0, 0);
    private float angularVel = 0;

    void Awake()
    {
        // Find track object so velocity can be fed to car controller
        track = GameObject.Find("MainTrack");
        if (track == null)
        {
            Debug.Log("The track object was not found.");
        }
    }

    void Start()
    {
        InvokeRepeating("UpdateGoal", 0f, 10f);
        InvokeRepeating("CalculateControls", 0f, 1 / Constants.targetHz);

        //Get dimensions of car sprite
        float length = GetComponentInChildren<Renderer>().bounds.size.x;
        float width = GetComponentInChildren<Renderer>().bounds.size.y;

        carController = new CarController(length);
        steerSys = new FirstOrderSystem(Constants.carSimSteeringK, Constants.carSimSteeringTau, Constants.targetHz, 0f);
        throttleSys = new AsymmetricFirstOrderSystem(Constants.carSimVelK, Constants.carSimVelIncreaseTau, Constants.carSimVelDecreaseTau, Constants.targetHz, 0f);

        // Set initial car state
        carState = new CarState(0, transform.position.z, transform.position.x, 0, length / 2, length / 2);
    }

    void CalculateControls()
    {
        // Receive feedback of the current treadmill velocity
        carController.treadmillVelCallback(track);      // temporary

        //temporary
        Pose convertedCoordinatesPose = new Pose(new Vector3(transform.position.z, transform.position.x, 0), transform.rotation);
        Vector2 convertedCoordinatesLinearVel = new Vector2(linearVel.z, linearVel.x);
        CarController.CarCommand command = carController.syncCallback(convertedCoordinatesPose, convertedCoordinatesLinearVel, angularVel);

        //TODO - figure out how to use throttle and steer to move the car
        carCommandCallback(command);

        UpdateCar();
    }

    // TODO - fix this once using ROS#
    void carCommandCallback(CarController.CarCommand command)
    {
        steer = steerSys.Output(command.steer);
        throttle = throttleSys.Output(command.throttle);
    }

    private void UpdateGoal()
    {
        //float x = Mathf.Clamp(Random.Range(transform.position.x - 3, transform.position.x + 3), 0, 20);
        //float z = Mathf.Clamp(Random.Range(transform.position.z - 20, transform.position.x + 20), 0, 40);
        float x = 5f;
        float z = 20f;
        Pose goal = new Pose(new Vector3(z, x, 0), transform.rotation);
        Debug.Log(string.Format("companion car goal: ({0}, {1})", z, x));
        carController.goalPoseCallback(goal);
    }

    void UpdateCar()
    {
        float dt = (1 / Constants.targetHz);
        float trackVel = track.gameObject.GetComponent<TrackSpeedUpdater>().vel;

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
}


