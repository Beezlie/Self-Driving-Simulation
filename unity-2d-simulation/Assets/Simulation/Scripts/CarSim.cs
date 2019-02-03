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
    private float carHeading;       // in radians
    private Vector2 linearVel;
    private float angularVel;
    private float carWidth;
    private float carLength;

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

        carController = new CarController();
        steerSys = new FirstOrderSystem(Constants.carSimSteeringK, Constants.carSimSteeringTau, Constants.targetHz, 0f);
        throttleSys = new AsymmetricFirstOrderSystem(Constants.carSimVelK, Constants.carSimVelIncreaseTau, Constants.carSimVelDecreaseTau, Constants.targetHz, 0f);

        // Get dimensions of car object
        Vector3 carSize = GetComponent<Renderer>().bounds.size;
        carWidth = carSize.y;
        carLength = carSize.x;
        Debug.Log(string.Format("Car length: {0}", carLength));

        // Set initial car data
        carHeading = Constants.neutralHeading;
        linearVel = new Vector3(0, 0, 0);
        angularVel = 0;
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

        Calculate();
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
        goal = new Pose(new Vector3(x, transform.position.y, 0), transform.rotation);   
        Debug.Log(string.Format("New Goal X: {0}", x));
        carController.goalPoseCallback(goal);
    }

    //Kinematic bicycle model
    void Calculate()
    {
        // TODO - how do I factor time into all of this???
        float dt = (1 / Constants.targetHz);   

        float lr = carLength / 2;
        float lf = carLength / 2;
        float a = throttle * 1;     // think need to multiply throttle by some constant accel
        float slipAngle = Mathf.Atan((lr / (lf + lr)) * Mathf.Tan(steer));
        float dv = a;   // might be a problem here  
        float dx = linearVel.x * Mathf.Cos(carHeading + slipAngle);
        float dy = linearVel.x * Mathf.Sin(carHeading + slipAngle);
        float dh = (linearVel.x / lr) * Mathf.Sin(slipAngle);

        Debug.Log(string.Format("lr = lf: {0}", lr));
        Debug.Log(string.Format("slip angle: {0}", slipAngle));
        Debug.Log(string.Format("dx: {0}", dx));
        Debug.Log(string.Format("dy: {0}", dy));
        Debug.Log(string.Format("dh: {0}", dh));
        Debug.Log(string.Format("dv: {0}", dv));

        // TODO - make sure all equations/calculations correct
        //Remember: dx, dy = change in distance over time (velocity)
        // dh = change in inertial heading
        // dv = change in speed over time (acceleration)

        // how to factor in constant velocity from treadmill?  
        // right now since acceleration is always +ve, if linearVel.x == trackVel, it will just constanly move forward
        float trackVel = track.gameObject.GetComponent<TrackSim>().vel;
        linearVel = new Vector3(linearVel.x + dv - trackVel, 0, 0);
        Debug.Log(string.Format("New vel: {0}", linearVel.x));

        // I think this is wrong - need to figure out what they are referring to as angular vel (rotation of heading or about some circle's radius)
        //angularVel = dh;

        carHeading = carHeading + dh ;  // is this correct?
        Debug.Log(string.Format("New heading: {0}", carHeading));

        Vector3 newPos = new Vector3(transform.position.x + dx, transform.position.y + dy, 0);
        Quaternion newRot = transform.rotation;
        newRot.eulerAngles = new Vector3(0, 0, Constants.neutralHeading + carHeading * Mathf.Rad2Deg);  // is this right?  Will it only rotate heading in one direction?
        transform.position = newPos;
        transform.rotation = newRot;
    }
}

