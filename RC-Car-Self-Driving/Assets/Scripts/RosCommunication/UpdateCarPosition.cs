using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;

public class UpdateCarPosition : MonoBehaviour {

    PoseStampedSubscriber sub;
    private float wheelOffset = 0.8f;       // offset to apply to y-coord of car so bottom of wheels touch track
    private float waypointScaling = 20f;    //real xMax = 2, yMax = 1, so scale to match virtual track - also in sim y = x and x = y

    // Use this for initialization
    void Start () {
        sub = gameObject.GetComponent(typeof(PoseStampedSubscriber)) as PoseStampedSubscriber;
    }

    // Update is called once per frame
    void Update()
    {
        // update position of car in game
        float x = sub.position.x * waypointScaling + 23;
        float z = sub.position.z * waypointScaling;
        float y = wheelOffset;
        Vector3 carPos = new Vector3(x, y, z);
        transform.position = carPos;
        transform.rotation = sub.rotation;
    }
}
