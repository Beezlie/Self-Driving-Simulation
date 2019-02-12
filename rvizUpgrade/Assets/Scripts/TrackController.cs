using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;

public class TrackController : MonoBehaviour {

    private float vel;
    private Float64Subscriber trackVelSubscriber;

    void Start () {
        trackVelSubscriber = gameObject.GetComponent(typeof(Float64Subscriber)) as Float64Subscriber;
    }

    void Update () {
        // Get track velocity from subscriber
        vel = trackVelSubscriber.messageData;
        Debug.Log(vel);
        
        // Move the track in the x direction
        float offset = Time.time * vel;
        GetComponent<Renderer>().material.mainTextureOffset = new Vector2(0, offset);
    }
}
