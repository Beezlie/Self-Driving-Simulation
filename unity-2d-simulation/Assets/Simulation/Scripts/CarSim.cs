using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//references https://bitbucket.org/dshin-uwaterloo/adas-car-on-treadmill-v2/src/master/src/vrep_sim/src/sim_car.cpp
public class CarSim : MonoBehaviour
{
    // Car Controls
    private CarController carController;
    private FirstOrderSystem steerSys;
    private AsymmetricFirstOrderSystem throttleSys;
    private float steer;        // in radians       
    private float throttle;     // in percentage
    private GameObject track;

    private Pose goal;      //temporary

    // Car info
    private CarState carState;
    private Vector3 linearVel = new Vector3(0, 0, 0);
    private float angularVel = 0;

    void Awake()
    {
        // Find track object so velocity can be fed to car controller
        track = GameObject.Find("Track");
        if (track == null) {
            Debug.Log("The track object was not found.");
        }
    }

    void Start()
    {
        InvokeRepeating("ModifyGoal", 0f, 10f);        //for testing to set goal
        InvokeRepeating("CalculateControls", 0f, 1 / Constants.targetHz);

        //Get dimensions of car sprite
        float length = GetComponent<SpriteRenderer>().bounds.size.x;
        float width = GetComponent<SpriteRenderer>().bounds.size.y;
        Debug.Log(string.Format("car length: {0}", length));

        carController = new CarController(length);
        steerSys = new FirstOrderSystem(Constants.carSimSteeringK, Constants.carSimSteeringTau, Constants.targetHz, 0f);
        throttleSys = new AsymmetricFirstOrderSystem(Constants.carSimVelK, Constants.carSimVelIncreaseTau, Constants.carSimVelDecreaseTau, Constants.targetHz, 0f);

        // Set initial car state
        carState = new CarState(0, transform.position.x, transform.position.y, 0, length / 2, length / 2);
    }

    void CalculateControls()
    {
        // Receive feedback of the current treadmill velocity
        carController.treadmillVelCallback(track);      // temporary

        //temporary
        CarController.CarCommand command = carController.syncCallback(new Pose(transform.position, transform.rotation), linearVel, angularVel);

        //TODO - figure out how to use throttle and steer to move the car
        carCommandCallback(command);
        Debug.Log(string.Format("Steer after passing through sys: {0}", steer));
        Debug.Log(string.Format("Throttle after passing through sys: {0}", throttle));

        UpdateCar();
    }

    // TODO - fix this once using ROS#
    void carCommandCallback(CarController.CarCommand command)
    {
        steer = steerSys.Output(command.steer);
        throttle = throttleSys.Output(command.throttle);
    }

    // For Testing
    private void ModifyGoal()
    {
        float x = Random.Range(10, 10);
        float y = 3;
        goal = new Pose(new Vector3(x, y, 0), transform.rotation);
        Debug.Log(string.Format("New Goal X: {0}", x));
        carController.goalPoseCallback(goal);
    }

    void UpdateCar()
    {
        float dt = (1 / Constants.targetHz);
        float trackVel = track.gameObject.GetComponent<TrackSim>().vel;

        //Update bicycle model
        carState.UpdateState(throttle, steer, dt, trackVel);

        //Update the state of the car sprite
        transform.position = new Vector3(carState.x, carState.y);
        float newAngle = (carState.psi * Mathf.Rad2Deg);// + Constants.neutralHeading;
        Debug.Log(string.Format("newAngle: {0}", newAngle));
        Quaternion rotation = Quaternion.Euler(0, 0, (carState.psi * Mathf.Rad2Deg));// + Constants.neutralHeading);
        Debug.Log(string.Format("rotation quaternion: {0}", rotation));
        transform.rotation = rotation;

        linearVel = new Vector3(carState.dx, carState.dy, 0);
        angularVel = carState.dv;
        Debug.Log(string.Format("Linear vel X: {0}, Y {1}", linearVel.x, linearVel.y));
    }
}

