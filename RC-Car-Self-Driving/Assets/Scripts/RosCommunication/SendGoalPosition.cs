using UnityEngine;
using RosSharp.RosBridgeClient;

public class SendGoalPosition : MonoBehaviour {
    private PoseStampedPublisher goalPublisher;
	
	private void Start () {
        // Attach pose stamped publisher to object
        goalPublisher = gameObject.AddComponent(typeof(PoseStampedPublisher)) as PoseStampedPublisher;
        goalPublisher.Topic = "/car/1/pose";

        // Create ROS Connector to establish websocket
        RosConnector rosConnector = gameObject.AddComponent(typeof(RosConnector)) as RosConnector;
        rosConnector.RosBridgeServerUrl = "ws://129.97.69.100:9090";    //lab base station computer IP address
        rosConnector.Protocol = RosConnector.Protocols.WebSocketSharp;
	}
	
	// Called once per frame
	private void Update () {
        //Publish this objects position + rotation to RC car
        goalPublisher.PublishedTransform = transform;
	}

    public void SetGoal(Pose goal) {
        transform.position = new Vector3(goal.position.x, goal.position.y, goal.position.z);
        transform.rotation = new Quaternion(goal.rotation.x, goal.rotation.y, goal.rotation.z, goal.rotation.w);
    }
}


