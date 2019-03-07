using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//references https://bitbucket.org/dshin-uwaterloo/adas-car-on-treadmill-v2/src/master/src/vrep_sim/src/sim_car.cpp
public class TuningCarInterface : MonoBehaviour
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
        if (track == null) {
            Debug.Log("The track object was not found.");
        }
    }

    void Start()
    {
        InvokeRepeating("UpdateGoal", 0f, 1f);        //for testing to set goal
        InvokeRepeating("CalculateControls", 0f, 1 / Constants.targetHz);

        //Get dimensions of car sprite
        float length = GetComponentInChildren<Renderer>().bounds.size.x;
        float width = GetComponentInChildren<Renderer>().bounds.size.y;
        Debug.Log(string.Format("car length: {0}", length));
        Debug.Log(string.Format("car width: {0}", width));

        carController = new CarController(length);
        steerSys = new FirstOrderSystem(Constants.carSimSteeringK, Constants.carSimSteeringTau, Constants.targetHz, 0f);
        throttleSys = new AsymmetricFirstOrderSystem(Constants.carSimVelK, Constants.carSimVelIncreaseTau, Constants.carSimVelDecreaseTau, Constants.targetHz, 0f);

        // Set initial car state
        // TODO: use the car's current direction as psi? DEVANSH
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
        Debug.Log(string.Format("Steer before passing through sys: {0}", command.steer));
        Debug.Log(string.Format("Throttle before passing through sys: {0}", command.throttle));

        UpdateCar();
    }

    // TODO - fix this once using ROS#
    void carCommandCallback(CarController.CarCommand command)
    {
        steer = steerSys.Output(command.steer);
        throttle = throttleSys.Output(command.throttle);
    }

    // For Testing
    private void UpdateGoal()
    {
        Pose goal = new Pose(new Vector3(Constants.goalCoordinates.z, Constants.goalCoordinates.x, 0), transform.rotation);
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
        Debug.Log(string.Format("newAngle: {0}", newAngle));
        Quaternion rotation = Quaternion.Euler(0, (carState.psi * Mathf.Rad2Deg), 0);   //has this changed now???
        Debug.Log(string.Format("rotation quaternion: {0}", rotation));
        transform.rotation = rotation;

        linearVel = new Vector3(carState.dx, 0, carState.dz);
        angularVel = carState.dv;       //is this right???
        Debug.Log(string.Format("Linear vel X: {0}, Z {1}", linearVel.x, linearVel.z));
        Debug.Log(string.Format("Steer: {0}, Throttle: {1}", steer, throttle));
        Debug.Log(string.Format("Psi: {0}", carState.psi));
    }
}

