using UnityEngine;

//references https://bitbucket.org/dshin-uwaterloo/adas-car-on-treadmill-v2/src/master/src/vrep_sim/src/sim_treadmill.cpp
public class TrackSim : MonoBehaviour
{
    private float speedGoal;
    public float vel = 0f;
    private float elapsed = 0f;
    private float throttle;

    private TrackController trackController;
    private AsymmetricFirstOrderSystem sys;

    private void Start()
    {
        InvokeRepeating("ModifySpeed", 0f, 10f);        //for testing
        InvokeRepeating("CalculateControls", 0f, 1 / Constants.targetHz);

        trackController = new TrackController();
        sys = new AsymmetricFirstOrderSystem(Constants.trackSimK, Constants.trackSimIncreaseTau, Constants.trackSimDecreaseTau, Constants.targetHz, 0f);
    }

    private void CalculateControls()
    {
        //Debug.Log(string.Format("track throttle: {0}", throttle));
        vel = sys.Output(throttle);
        //Debug.Log(string.Format("track vel: {0}", vel));
        throttle = trackController.velCallback(vel);

        //move the track in the x direction
        float offset = Time.time * vel;
        GetComponent<Renderer>().material.mainTextureOffset = new Vector2(0, -offset);
    }

    // For Testing
    private void ModifySpeed()
    {
        speedGoal = 0.01f;
        throttle = trackController.commandVelCallback(speedGoal);
    }
}