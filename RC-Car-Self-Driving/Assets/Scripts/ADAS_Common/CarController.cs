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

    //temporary
    public float desiredVel = 0;

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

    public CarController(float axleDistance)
    {
        velController = new PIDController(Constants.pVel, Constants.iVel, Constants.dVel, Constants.throttleCommandMin, Constants.throttleCommandMax);
        posController = new PIDController(Constants.pPos, Constants.iPos, Constants.dPos, Constants.velMin, Constants.velMax);
        headController = new StanleyController(Constants.kpHeading, Constants.kdHeading, Constants.kCrosstrack, Constants.velDamping, axleDistance / 2);
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
    public CarCommand syncCallback(Pose pose, Vector2 linearVel, float angularVel)
    {
        if (!goalReceived)
        {
            return new CarCommand(0, 0);
        }

        //TODO - check to make sure this is doing same thing as before
        //Debug.Log(string.Format("Observed pose of car: {0}", pose));

        // Velocity at which we'd like to approach the goal.
        float velCommand = posController.commandStep(0, pose.position.x, 1 / Constants.targetHz, true);

        // Set goal to approach the goal AND maintain position.
        desiredVel = velCommand + treadmillVel;
        //Debug.Log(string.Format("Velocity to approach the goal with after factoring n treadmill vel: {0}", desiredVel));
        velController.setGoal(desiredVel);

        // Explicit decision: No reset of D term calculation. It's not user-controlled and is assumed to be "reasonably continuous."
        /*
            * Now calculate the throttle required for the specified velocity.
            * NOTE: Accumulate I term iff we're close to the goal.
        */
        float throtCommand = velController.commandStep(treadmillVel * velKFeedforward,
                                                        linearVel.x + treadmillVel,
                                                        1 / Constants.targetHz,
                                                        Mathf.Abs(pose.position.x - posController.getGoal()) < posIThreshold);

        float headCommand = headController.commandStep(pose,
                                                        new Vector3(treadmillVel + linearVel.x, linearVel.y, 0),
                                                        angularVel,
                                                        1 / Constants.targetHz);

        //TODO - now have to send throttle and head commands somewhere to control the car
        return new CarCommand(headCommand, throtCommand);
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
    public void treadmillVelCallback(GameObject treadmill)
    {
        treadmillVel = treadmill.gameObject.GetComponent<TrackSpeedUpdater>().vel;
        //Debug.Log(string.Format("treadmill speed: {0}", treadmillVel));
    }

    private void resetCallback()
    {
        Debug.Log("Resetting controller.");
        posController.reset();
        velController.reset();
        headController.reset();
    }
}