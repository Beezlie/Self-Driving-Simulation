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

    private Rigidbody2D rb;
    private GameObject track;

    private Pose goal;      //temporary

    // Car info
    Vector2 carLocation;
    private float carHeading;       // in radians
    private Vector2 linearVel;
    private float angularVel;
    private float carWidth;
    private float carLength;
    private float wheelBase; // the distance between the two axles

    void Awake()
    {
        //TODO - maybe remove the rigidbody and do this by just moving the transform of the car realistically
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

        // Get dimensions of car object
        Vector3 carSize = GetComponent<Renderer>().bounds.size;
        carWidth = carSize.y;
        carLength = carSize.x;
        Debug.Log(string.Format("Car length: {0}", carLength));

        // Set initial car data
        carLocation = new Vector2(transform.position.x, transform.position.y);
        carHeading = Constants.neutralHeading;
        linearVel = new Vector3(0, 0, 0);
        angularVel = 0;
        wheelBase = carLength;  //TODO - make this more exact
    }

    void CalculateControls ()
    {
        // Receive feedback of the current treadmill velocity
        carController.treadmillVelCallback(track);      // temporary
        
        //temporary
        CarController.CarCommand command = carController.syncCallback(new Pose(transform.position, transform.rotation), linearVel, angularVel);       

        //TODO - figure out how to use throttle and steer to move the car
        carCommandCallback(command);
        Debug.Log(string.Format("Steer after passing through sys: {0}", steer));
        Debug.Log(string.Format("Throttle after passing through sys: {0}", throttle));

        //MoveCarLongitudinally();
        //MoveCarLaterally();
        //UpdateCarSprite();
        Calculate();
    }

    void MoveCarLongitudinally()
    {
        float trackVel = track.gameObject.GetComponent<TrackSim>().vel;

        // Use throttle to move the EgoCar in the forward direction
        float acceleration = 40;
        float dt = (1 / Constants.targetHz);
        float newVel = throttle * acceleration * dt + linearVel.x - trackVel;
        Debug.Log(string.Format("New vel: {0}", newVel));
        linearVel = new Vector2(newVel, 0);

        //rb.velocity = new Vector2(linearVel.x, 0);
    }

    //TODO - need to track and update linearVel.y for stanleycontroller
    void MoveCarLaterally()
    {
        // Find actual positions of the wheels
        Vector2 frontWheel = carLocation + wheelBase / 2 * new Vector2(Mathf.Cos(carHeading), Mathf.Sin(carHeading));
        Vector2 backWheel = carLocation - wheelBase / 2 * new Vector2(Mathf.Cos(carHeading), Mathf.Sin(carHeading));

        // Calculate the amount that each wheel should move forward by
        float dt = (1 / Constants.targetHz);
        backWheel += linearVel.x * dt * new Vector2(Mathf.Cos(carHeading), Mathf.Sin(carHeading));
        frontWheel += linearVel.x * dt * new Vector2(Mathf.Cos(carHeading + steer), Mathf.Sin(carHeading + steer));

        //The new car position can be calculated by averaging the two new wheel positions. 
        carLocation = (frontWheel + backWheel) / 2;

        //The new car heading can be found by calculating the angle of the line between the two new wheel positions
        carHeading = Mathf.Atan2(frontWheel.y - backWheel.y, frontWheel.x - backWheel.x);
        Debug.Log(string.Format("Car heading: {0}", carHeading));

        // TODO - Set angularVel - use change in car heading.  so it can be passed to stanley controller
        // TODO - Set linearVel.y so it can be passed to stanley controller

    }

    // Update car sprite with new position and heading
    void UpdateCarSprite()
    {

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
        int x = Random.Range(7, 7);
        int y = Random.Range(6, 6);
        goal = new Pose(new Vector3(x, y, 0), transform.rotation);   
        Debug.Log(string.Format("New Goal X: {0}, Y: {1}", x, y));
        carController.goalPoseCallback(goal);
    }

    //Kinematic bicycle model
    void Calculate()
    {
        float lr = carLength / 2;
        float lf = carLength / 2;
        float a = throttle * 0.01f;

        float slipAngle = Mathf.Atan((lr / (lf + lr)) * Mathf.Tan(steer));
        float dt = (1 / Constants.targetHz);    // I think this should be correct since function is called this often
        float dv = a;
        float v = linearVel.x + dv * dt;
        float dx = v * Mathf.Cos(carHeading + slipAngle);
        float dy = v * Mathf.Sin(carHeading + slipAngle);
        float dh = (v / lr) * Mathf.Sin(slipAngle);

        Debug.Log(string.Format("lr = lf: {0}", lr));
        Debug.Log(string.Format("slip angle: {0}", slipAngle));
        Debug.Log(string.Format("dx: {0}", dx));
        Debug.Log(string.Format("dy: {0}", dy));
        Debug.Log(string.Format("dh: {0}", dh));
        Debug.Log(string.Format("dv: {0}", dv));

        // TODO - make sure all equations/calculations correct
        //Remember: dx, dy = change in distance over time (velocity)
        // dh = change in rotation over time (angular velocity)
        // dv = change in speed over time (acceleration)

        //linearVel = new Vector2(linearVel.x + dx, 0);
        linearVel = new Vector2(linearVel.x + dx, linearVel.y + dy);
        Debug.Log(string.Format("New vel: {0}", linearVel.x));

        angularVel = dh;
        carHeading = carHeading + dh * dt;
        Debug.Log(string.Format("New heading: {0}", carHeading));

        //Vector3 newPos = new Vector3(transform.position.x + dx * dt, transform.position.y, 0);
        Vector3 newPos = new Vector3(transform.position.x + dx * dt, transform.position.y + dy * dt, 0);
        Quaternion newRot = transform.rotation;
        newRot.eulerAngles = new Vector3(0, 0, Constants.neutralHeading + carHeading * Mathf.Rad2Deg);
        transform.position = newPos;
        transform.rotation = newRot;
    }
}

