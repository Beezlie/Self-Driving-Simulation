using UnityEngine;

// references https://bitbucket.org/dshin-uwaterloo/adas-car-on-treadmill-v2/src/master/src/car_controller/src/car_controller.cpp
public class CarController
{
    // Controllers
    private PIDController posController;
    private PIDController velController;
    private StanleyController headController;

    private float velKFeedforward;
    private float treadmillVel;
    private float posIThreshold;
    private float discontinuityThreshold;
    private bool goalReceived;

    public CarCommand command;      //temporary

    public class PIDParams
    {
        public float p;
        public float i;
        public float d;

        public PIDParams(float p, float i, float d)
        {
            this.p = p;
            this.i = i;
            this.d = d;
        }
    }

    public class SaturationParams
    {
        public float min;
        public float max;

        public SaturationParams(float min, float max)
        {
            this.min = min;
            this.max = max;
        }
    }

    public class CarCommand
    {
        public float steer;
        public float throttle;

        public CarCommand(float steer, float throttle)
        {
            this.steer = steer;
            this.throttle = throttle;
        }
    }

    public CarController()
    {
        velController = new PIDController(Constants.pVel, Constants.iVel, Constants.dVel, Constants.throttleCommandMin, Constants.throttleCommandMax);
        posController = new PIDController(Constants.pPos, Constants.iPos, Constants.dPos, Constants.velMin, Constants.velMax);
        headController = new StanleyController(Constants.kpHeading, Constants.kdHeading, Constants.kCrosstrack, Constants.velDamping, Constants.axleDistance);
        velKFeedforward = Constants.kFeedForward;
        discontinuityThreshold = Constants.discontinuityThreshold;
        posIThreshold = Constants.iThreshold;
        goalReceived = false;
    }

    protected void setPosPIDParams(PIDParams posParams)
    {
        posController.p = posParams.p;
        posController.i = posParams.i;
        posController.d = posParams.d;
    }

    protected void setVelSatParams(SaturationParams velSatParams)
    {
        // On position control as it defines the max velocity command
        posController.setSaturation(velSatParams.min, velSatParams.max);
    }

    protected void setVelPIDParams(PIDParams velParams)
    {
        velController.p = velParams.p;
        velController.i = velParams.i;
        velController.d = velParams.d;
    }

    protected void setVelKFeedforward(float kFeedforward)
    {
        velKFeedforward = kFeedforward;
    }

    protected void setKPHeading(float kPHeading)
    {
        headController.setKPHeading(kPHeading);
    }

    protected void setKDHeading(float kDHeading)
    {
        headController.setKDHeading(kDHeading);
    }

    protected void setKCrosstrack(float kCrosstrack)
    {
        headController.setKCrosstrack(kCrosstrack);
    }

    protected void setVelDamping(float velDamping)
    {
        headController.setVelDamping(velDamping);
    }

    //TODO - make this private once ROS# implemented and pass in ROS Pose instead of system Pose
    public void syncCallback(Pose pose, Vector3 linearVel, float angularVel)
    {
        if (!goalReceived)
        {
            return;
        }

        //TODO - check to make sure this is doing same thing as before
        Pose observedPose = new Pose(pose.position, pose.rotation);

        // Velocity at which we'd like to approach the goal.
        // TODO - SOURCE CODE WAS MISSING LAST ARGUMENT - USED FALSE HERE BUT NOT SURE IF CORRECT
        float velCommand = posController.commandStep(0, observedPose.position.x, Time.deltaTime, false);

        // Set goal to approach the goal AND maintain position.
        float desiredVel = velCommand + treadmillVel;
        velController.setGoal(desiredVel);

        // Explicit decision: No reset of D term calculation. It's not user-controlled and is assumed to be "reasonably continuous."
        /*
            * Now calculate the throttle required for the specified velocity.
            * NOTE: Accumulate I term iff we're close to the goal.
        */
        //TODO - double check anywhere I used deltaTime to make sure that's actually the correct way to do this
        float throtCommand = velController.commandStep(treadmillVel * velKFeedforward,
                                                        linearVel.x + treadmillVel,
                                                        Time.deltaTime,
                                                        Mathf.Abs(pose.position.x - posController.getGoal()) < posIThreshold);

        //TODO - double check anywhere I used deltaTime to make sure that's actually correct
        float headCommand = headController.commandStep(observedPose,
                                                        new Vector3(treadmillVel + linearVel.x, linearVel.y, 0),
                                                        angularVel,
                                                        Time.deltaTime);

        //TODO - now have to send throttle and head commands somewhere to control the car
        //THIS SHOULD BE WHAT ACTUALLY MOVES IT
        command =  new CarCommand(headCommand, throtCommand);
    }

    //TODO - make this private once ROS# implemented
    public void goalPoseCallback(Pose goal)
    {
        headController.setGoal(goal);

        float prevGoalX = posController.getGoal();
        float newGoalX = goal.position.x;
        if (goalReceived && Mathf.Abs(prevGoalX - newGoalX) > discontinuityThreshold) {
            Debug.Log("Large path discontinuity detected. Resetting Controller.");
            posController.reset();
            velController.reset();
        }

        posController.setGoal(newGoalX);

        goalReceived = true;
    }

    // TODO - fix this once using ROS# and make private
    public void treadmillVelCallback()
    {
        //treadmillVel = twist.linear.x;
        //TODO - get the value from the track
        treadmillVel = 1;       // temporary
    }

    private void resetCallback()
    {
        Debug.Log("Resetting controller.");
        posController.reset();
        velController.reset();
        headController.reset();
    }
}