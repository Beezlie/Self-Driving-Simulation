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

        goal = new Pose(new Vector3(0,0,0), new Quaternion(0,0,0,0));       //temporary
    }

    void Update()
    {
        carController.treadmillVelCallback(new RosSharp.RosBridgeClient.Messages.Geometry.Twist());      // temporary
        // TODO - get pose and send to controller goal pose callback
        // TODO - send correct pose
        carController.goalPoseCallback(goal);       //temporary

        // TODO - sync car controller with position of sprite
        Vector3 linearVel = new Vector3(rb.velocity.x, rb.velocity.y, 0);
        //TODO - figure out how to get angular velocity to a Twist
        carController.syncCallback(new Pose(transform.position, transform.rotation), new RosSharp.RosBridgeClient.Messages.Geometry.Twist());        //temporary

        // TODO - get command from car controller
        command = carController.command;        //temporary

        //TODO - figure out how to use throttle and head to move the car
        Debug.Log("Throttle");
        Debug.Log(command.throttle);
        Debug.Log("Steer");
        Debug.Log(command.steer);
    }

    // TODO - fix this once using ROS#
    void carCommandCallback()
    {
        steer = steerSys.Output(command.steer);
        throttle = throttleSys.Output(command.throttle);
    }
}

