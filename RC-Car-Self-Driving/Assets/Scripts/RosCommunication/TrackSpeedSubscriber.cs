using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;
using UnityEngine.SceneManagement;

public class TrackSpeedSubscriber : MonoBehaviour {

    private float vel = 0.9f;
    private bool isTrainingMode = false;
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
        // Reference to the current scene
        Scene currentScene = SceneManager.GetActiveScene();
        string sceneName = currentScene.name;
        if (currentScene.name == "AgentTrainingEnvironment")
        {
            isTrainingMode = true;
        }

        // Initialize system
        trackVelSubscriber = gameObject.GetComponent(typeof(TwistStampedSubscriber)) as TwistStampedSubscriber;
        trackController = new TrackController();
        sys = new AsymmetricFirstOrderSystem(Constants.trackSimK, Constants.trackSimIncreaseTau, Constants.trackSimDecreaseTau, Constants.targetHz, 0f);
    }

    private void Update () {
        if (!isTrainingMode)
        {
            // Start control loop if new velocity command given
            if (linearVel != trackVelSubscriber.linearVel)
            {
                linearVel = trackVelSubscriber.linearVel;
                throttle = trackController.SetGoalVelocity(trackVelSubscriber.linearVel.z);
            }
            else
            {
                vel = sys.Output(throttle);
                throttle = trackController.GetTreadmillThrottle(vel);
            }
        }
    }
}
