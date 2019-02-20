using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;

public class sendWaypoint : MonoBehaviour {
    PoseStampedPublisher pub;

    /*
	// Use this for initialization
	void Start () {
        pub = gameObject.GetComponent(typeof(PoseStampedPublisher)) as PoseStampedPublisher;
        pub.Topic = "/car/1/nav/goal";
    }
	
	// Update is called once per frame
	void Update () {
        transform.position = new Vector3(-0.5f, 0f, 0.5f);
        transform.rotation = new Quaternion(0, 0, 0, 1f);
        //z = y
        //y = -x
        //x = z
    }
    */
}
