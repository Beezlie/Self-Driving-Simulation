using UnityEngine;
using RosSharp.RosBridgeClient;

//references https://bitbucket.org/dshin-uwaterloo/adas-car-on-treadmill-v2/src/master/src/vrep_sim/src/sim_treadmill.cpp
public class TrackSim : MonoBehaviour
{
    public float speed;
    private float elapsed = 0f;
    Vector2 offset;

    private TrackController trackController;
    private AsymmetricFirstOrderSystem sys;

    private void Start()
    {
        trackController = new TrackController();
        sys = new AsymmetricFirstOrderSystem(Constants.trackSimK, Constants.trackSimIncreaseTau, Constants.trackSimDecreaseTau, Constants.targetHz, 0f);
    }

    private void Update()
    {
        float throttle = trackController.commandVelCallback(speed);
        float vel = sys.Output(throttle);

        //move the track in the x direction
        offset = new Vector2(0, Time.time * vel);
        GetComponent<Renderer>().material.mainTextureOffset = offset;
    }
}