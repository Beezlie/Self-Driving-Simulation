using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//references https://bitbucket.org/dshin-uwaterloo/adas-car-on-treadmill-v2/src/master/src/vrep_sim/src/sim_car.cpp
public class CarSim : MonoBehaviour
{

    private CarController carController;
    private FirstOrderSystem steerSys;
    private AsymmetricFirstOrderSystem throttleSys;
    private float steer;
    private float throttle;

    private Rigidbody2D rb;
    private GameObject track;

    private Pose goal;      //temporary

    void Awake()
    {
        rb = (Rigidbody2D)GetComponent(typeof(Rigidbody2D));
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

        carController = new CarController();
        steerSys = new FirstOrderSystem(Constants.carSimSteeringK, Constants.carSimSteeringTau, Constants.targetHz, 0f);
        throttleSys = new AsymmetricFirstOrderSystem(Constants.carSimVelK, Constants.carSimVelIncreaseTau, Constants.carSimVelDecreaseTau, Constants.targetHz, 0f);
    }

    void CalculateControls ()
    {
        // Receive feedback of the current treadmill velocity
        carController.treadmillVelCallback(track);      // temporary

        // TODO - make sure way I am getting linear and angular velocity is right
        Vector3 linearVel = new Vector3(rb.velocity.x, rb.velocity.y);
        Debug.Log(string.Format("Linear vel x: {0}, Linear vel y: {1}", linearVel.x, linearVel.y));
        float angularVel = rb.angularVelocity;
        Debug.Log(string.Format("Angular vel: {0}", angularVel));
        //temporary
        CarController.CarCommand command = carController.syncCallback(new Pose(transform.position, transform.rotation), linearVel, angularVel);       

        //TODO - figure out how to use throttle and head to move the car
        Debug.Log(string.Format("Throttle before passing through sys: {0}", command.throttle));
        //Debug.Log(string.Format("Steer: {0}", command.steer));

        carCommandCallback(command);
        Debug.Log(string.Format("Throttle after passing through sys: {0}", throttle));

        // Use throttle to move the EgoCar in the forward direction
        //DOES ANY OF THIS MAKE SENSE???
        float trackVel = track.gameObject.GetComponent<TrackSim>().vel;
        float newVel = throttle * (1 / Constants.targetHz) - trackVel;
        Debug.Log(string.Format("New vel: {0}", newVel));
        rb.velocity = new Vector2(rb.velocity.x + newVel, 0);
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
        int x = Random.Range(6, 6);
        goal = new Pose(new Vector3(x, 0, 0), new Quaternion(0, 0, 0, 0));   
        Debug.Log(string.Format("New Goal X: {0}", goal.position.x));
        carController.goalPoseCallback(goal);
    }
}

