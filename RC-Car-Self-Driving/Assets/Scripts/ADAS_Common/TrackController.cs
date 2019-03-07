using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//references https://bitbucket.org/dshin-uwaterloo/adas-car-on-treadmill-v2/src/master/src/treadmill_controller/src/treadmill_controller.cpp
public class TrackController {
    private PIDController controller;

    // Linear fitting coefficient for feedforward control
    private float slope;
    private float intercept;

    public TrackController()
    {
        slope = Constants.trackSlope;
        intercept = Constants.trackIntercept;
        controller = new PIDController(Constants.pTrack, Constants.iTrack, Constants.dTrack, Constants.throttleCommandMin, Constants.throttleCommandMax);
    }

    public float SetGoalVelocity(float goal)
    {
        controller.setGoal(goal);

        // Kickstart with the feedforward
        float feedforward = calculateFeedforward(controller.getGoal());

        float throttle = feedforward;
        return throttle;
    }

    public float GetTreadmillThrottle(float trackVel)
    {
        float throt;
        float c = controller.commandStep(calculateFeedforward(controller.getGoal()), trackVel, 1 / Constants.targetHz, true);
        throt = c;
        return throt;
    }

    private float calculateFeedforward(float requestedVel)
    {
        if (requestedVel == 0) {
            return 0;
        }

        float throt = requestedVel * slope + intercept;

        if (throt < 0) {
            // Unsupported nonzero throttle - make it 1%.
            return 1;
        }

        return throt;
    }
}

