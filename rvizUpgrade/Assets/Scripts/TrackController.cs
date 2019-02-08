using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;

public class TrackController : MonoBehaviour {

    private float vel;
    private Float64Subscriber trackVelSubscriber;
    private RosConnector rosConnector;

    void Start () {
        //create ros connector
        rosConnector = gameObject.AddComponent(typeof(RosConnector)) as RosConnector;
        rosConnector.RosBridgeServerUrl = "ws://10.215.110.153:9090";
        rosConnector.Timeout = 10;
        rosConnector.Protocol = RosConnector.Protocols.WebSocketSharp;
        rosConnector.Awake();

        while (rosConnector.RosSocket == null)
        {
            // wait for RosSocket to be created
        }

        //create track vel subscriber
        trackVelSubscriber = gameObject.AddComponent(typeof(Float64Subscriber)) as Float64Subscriber;
        trackVelSubscriber.Topic = "/treadmill/velocity";
        trackVelSubscriber.TimeStep = 10;
    }
	
	void Update () {
        // Get track velocity from subscriber
        vel = trackVelSubscriber.messageData;
        
        // Move the track in the x direction
        float offset = Time.time * vel;
        GetComponent<Renderer>().material.mainTextureOffset = new Vector2(0, offset);
    }
}
