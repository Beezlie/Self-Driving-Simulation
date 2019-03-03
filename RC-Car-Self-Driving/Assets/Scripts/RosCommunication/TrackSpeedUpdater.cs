using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;

public class TrackSpeedUpdater : MonoBehaviour {

    public float vel = 0f;
    private float throttle;
    private Vector3 linearVel;
    private TwistStampedSubscriber trackVelSubscriber;
    private TrackController trackController;
    private AsymmetricFirstOrderSystem sys;

    void Start () {
        trackVelSubscriber = gameObject.GetComponent(typeof(TwistStampedSubscriber)) as TwistStampedSubscriber;
        trackController = new TrackController();
        sys = new AsymmetricFirstOrderSystem(Constants.trackSimK, Constants.trackSimIncreaseTau, Constants.trackSimDecreaseTau, Constants.targetHz, 0f);
    }

    void Update () {
        // Start control loop if new velocity command given
        if (linearVel != trackVelSubscriber.linearVel) {
            linearVel = trackVelSubscriber.linearVel;
            throttle = trackController.commandVelCallback(trackVelSubscriber.linearVel.z);
        } else {
            vel = sys.Output(throttle);
            throttle = trackController.velCallback(vel);
        }
        
        // Move the track in the x direction
        float offset = Time.time * -1 * Mathf.Abs(vel);
        GetComponent<Renderer>().material.mainTextureOffset = new Vector2(0, offset);
    }
}
