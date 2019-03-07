using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;

public class TrackSpeedSubscriber : MonoBehaviour {

    private float vel = 0.8f;
    private float throttle;
    private Vector3 linearVel;
    private TwistStampedSubscriber trackVelSubscriber;
    private TrackController trackController;
    private AsymmetricFirstOrderSystem sys;

    public float GetVelocity()
    {
        return vel;
    }

    private void Start () {
        // Initialize system
        trackVelSubscriber = gameObject.GetComponent(typeof(TwistStampedSubscriber)) as TwistStampedSubscriber;
        trackController = new TrackController();
        sys = new AsymmetricFirstOrderSystem(Constants.trackSimK, Constants.trackSimIncreaseTau, Constants.trackSimDecreaseTau, Constants.targetHz, 0f);
    }

    private void Update () {
        /*
        // Start control loop if new velocity command given
        if (linearVel != trackVelSubscriber.linearVel) {
            linearVel = trackVelSubscriber.linearVel;
            throttle = trackController.SetGoalVelocity(trackVelSubscriber.linearVel.z);
        } else {
            vel = sys.Output(throttle);
            throttle = trackController.GetTreadmillThrottle(vel);
        }
        */
    }
}
