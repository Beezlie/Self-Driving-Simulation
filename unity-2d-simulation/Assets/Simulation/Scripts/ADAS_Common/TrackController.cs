using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;

//references https://bitbucket.org/dshin-uwaterloo/adas-car-on-treadmill-v2/src/master/src/treadmill_controller/src/treadmill_controller.cpp
public class TrackController {
    private PIDController controller;

    // Linear fitting coefficient for feedforward control
    private float slope;
    private float intercept;

    public float throttle;  //Temporary until ROS# communication implemented (then it gets published)

    public TrackController()
    {
        slope = Constants.trackSlope;
        intercept = Constants.trackIntercept;
        controller = new PIDController(Constants.pTrack, Constants.iTrack, Constants.dTrack, Constants.throttleCommandMin, Constants.throttleCommandMax);
    }

    //TODO - make this private void and publish throttle once ROS# communication implemented
    public float commandVelCallback(float goal)
    {
        controller.setGoal(goal);

        // Kickstart with the feedforward
        float feedforward = calculateFeedforward(controller.getGoal());

        throttle = feedforward;
        ///TODO publish throttle
        return throttle;
    }

    // I THINK THIS FUNCTION USES FEEDBACK FROM ENCODERS - THEREFORE IT ISNT NEEDED
    /*
    private void velCallback(Messages.Geometry.Twist twist)
    {
        float throt;
        // SOURCE CODE WAS MISSING LAST ARGUMENT - USED FALSE HERE BUT NOT SURE IF CORRECT
        float c = controller.commandStep(calculateFeedforward(controller.getGoal()), twist.linear.x, Time.deltaTime, false);
        throt = c;
        //TODO - use the throttle to control track speed
    }
    */

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

