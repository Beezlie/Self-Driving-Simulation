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
    float length;
    float width;

    public void ResetParams()
    {
        carController = new CarController(length);
        steerSys = new FirstOrderSystem(Constants.carSimSteeringK, Constants.carSimSteeringTau, Constants.targetHz, 0f);
        throttleSys = new AsymmetricFirstOrderSystem(Constants.carSimVelK, Constants.carSimVelIncreaseTau, Constants.carSimVelDecreaseTau, Constants.targetHz, 0f);
    }

    void Awake()
    {
        // Find track object so velocity can be fed to car controller
        track = GameObject.Find("TrackData");
        if (track == null) {
            Debug.Log("The track object was not found.");
        }
    }

    void Start()
    {
        InvokeRepeating("UpdateGoal", 0f, 1f);        //for testing to set goal
        InvokeRepeating("UpdateCar", 0f, 1 / Constants.targetHz);

        //Get dimensions of car sprite
        length = GetComponentInChildren<Renderer>().bounds.size.x;
        width = GetComponentInChildren<Renderer>().bounds.size.y;
        Debug.Log(string.Format("car length: {0}", length));
        Debug.Log(string.Format("car width: {0}", width));

        carController = new CarController(length);
        steerSys = new FirstOrderSystem(Constants.carSimSteeringK, Constants.carSimSteeringTau, Constants.targetHz, 0f);
        throttleSys = new AsymmetricFirstOrderSystem(Constants.carSimVelK, Constants.carSimVelIncreaseTau, Constants.carSimVelDecreaseTau, Constants.targetHz, 0f);

        // Set initial car state
        transform.position = new Vector3(Constants.goalCoordinates.x, 1, Constants.goalCoordinates.z);
        carState = new CarState(0, transform.position.z, transform.position.x, 0, length / 2, length / 2);
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
    }

    // Takes goal from Constants - makes PID tuning through UI easier
    private void UpdateGoal()
    {
        Pose goal = new Pose(new Vector3(Constants.goalCoordinates.z, Constants.goalCoordinates.x, 0), transform.rotation);
        carController.setNewGoalPosition(goal);
    }
}

