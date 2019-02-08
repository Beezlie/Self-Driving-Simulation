using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;

public class DetectCars : MonoBehaviour {

    private int numCars = 1;
    private List<PoseStampedSubscriber> carPoseSubscribers;
    private List<RosConnector> connectors;
    private List<GameObject> cars;
    private string[] carPrefabs = { "Car_1", "Car_2", "Car_3" };

    private float wheelOffset = 0.4f;       // offset to apply to y-coord of car so bottom of wheels touch track

    void Start () {
        carPoseSubscribers = new List<PoseStampedSubscriber>();
        connectors = new List<RosConnector>();
        cars = new List<GameObject>();
        for (int i = 0; i < numCars; i++)
        {
            //instantiate each car
            GameObject car = Instantiate(Resources.Load("Prefabs/" + carPrefabs[i], typeof(GameObject))) as GameObject;

            //create a ros connector for each car
            RosConnector rosConnector = car.AddComponent(typeof(RosConnector)) as RosConnector;
            rosConnector.RosBridgeServerUrl = "ws://10.215.110.153:9090";
            rosConnector.Timeout = 10;
            rosConnector.Protocol = RosConnector.Protocols.WebSocketSharp;
            rosConnector.Awake();
            connectors.Add(rosConnector);

            // create a subscriber for each car
            PoseStampedSubscriber carPosSubscriber = car.AddComponent(typeof(PoseStampedSubscriber)) as PoseStampedSubscriber;
            carPosSubscriber.Topic = "/car/" + i + "/pose/";
            carPosSubscriber.TimeStep = 10;
            carPoseSubscribers.Add(carPosSubscriber);

            cars.Add(car);
        }
    }
	
	void Update () {
		for (int i = 0; i < numCars; i++)
        {
            GameObject car = cars[i];
            if (carPoseSubscribers[i].position != null && carPoseSubscribers[i].rotation != null)
            {
                // update position of car in game
                car.transform.position = new Vector3(carPoseSubscribers[i].position.x, carPoseSubscribers[i].position.y + wheelOffset, carPoseSubscribers[i].position.z);
                car.transform.rotation = carPoseSubscribers[i].rotation;
            } else
            {
                //hide the car since it is not found on the physical track
                car.SetActive(false);
            }
        }
	}
}
