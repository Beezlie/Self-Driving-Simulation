using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//references https://bitbucket.org/dshin-uwaterloo/adas-car-on-treadmill-v2/src/master/src/vrep_sim/src/sim_car.cpp
public class CarSim : MonoBehaviour
{

    private CarController carController;
    private CarController.CarCommand command;           //temporary
    private FirstOrderSystem steerSys;
    private AsymmetricFirstOrderSystem throttleSys;
    private float steer;
    private float throttle;

    private Rigidbody2D rb;

    private Pose goal;      //temporary

    void Awake()
    {
        rb = (Rigidbody2D)GetComponent(typeof(Rigidbody2D));
    }

    void Start()
    {
        carController = new CarController();
        steerSys = new FirstOrderSystem(Constants.carSimSteeringK, Constants.carSimSteeringTau, Constants.targetHz, 0f);
        throttleSys = new AsymmetricFirstOrderSystem(Constants.carSimVelK, Constants.carSimVelIncreaseTau, Constants.carSimVelDecreaseTau, Constants.targetHz, 0f);
    }

    void Update()
    {
        goal = new Pose(new Vector3(-6, 0, 0), new Quaternion(0, 0, 0, 0));       //temporary

        carController.treadmillVelCallback();      // temporary
        // TODO - get pose and send to controller goal pose callback
        // TODO - send correct pose (pose = goal = waypoint I think)
        carController.goalPoseCallback(goal);       //temporary

        // TODO - make sure way I am getting linear and angular velocity is right
        Vector3 linearVel = new Vector3(rb.velocity.x, rb.velocity.y);
        float angularVel = rb.angularVelocity;
        carController.syncCallback(new Pose(transform.position, transform.rotation), linearVel, angularVel);        //temporary

        // TODO - get command from car controller
        command = carController.command;        //temporary

        //TODO - figure out how to use throttle and head to move the car
        Debug.Log(string.Format("Throttle before passing through sys: {0}", command.throttle));
        Debug.Log(string.Format("Steer: {0}", command.steer));

        carCommandCallback();
        Debug.Log(string.Format("Throttle after passing through sys: {0}", throttle));
    }

    // TODO - fix this once using ROS#
    void carCommandCallback()
    {
        steer = steerSys.Output(command.steer);
        throttle = throttleSys.Output(command.throttle);
    }
}

