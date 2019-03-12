using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;

public class CarPositionUpdater : MonoBehaviour {

    public int numCars = 3;
    public int egoCarNum = 3;

    private List<GameObject> cars;
    private string url = "ws://129.97.69.100:9090";
    private Vector3 scale = new Vector3(1f, 1f, 2.5f);
    private Vector3 initPos = new Vector3(1f, 1f, 1f);
    private float wheelOffset = 1f;       // offset to apply to y-coord of car so bottom of wheels touch track
    private float waypointScaling = 20f;    //real xMax = 2, yMax = 1, so scale to match virtual track - also in sim y = x and x = y

    void Start () {

        // Create car objects and subscriber/rosconnector for each car
        cars = new List<GameObject>();
        for (int i = 0; i < numCars; i++)
        {
            GameObject car = Instantiate(Resources.Load("Prefabs/Car_" + i)) as GameObject;
            car.name = "Car_" + i;
            car.transform.localScale = scale;
            car.transform.position = new Vector3(0f, 1f, 0f);
            car.SetActive(false);

            RosConnector rosConnector = car.AddComponent(typeof(RosConnector)) as RosConnector;
            rosConnector.Protocol = RosConnector.Protocols.WebSocketSharp;
            rosConnector.Timeout = 10;
            rosConnector.RosBridgeServerUrl = url;

            PoseStampedSubscriber poseStampedSubscriber = car.AddComponent(typeof(PoseStampedSubscriber)) as PoseStampedSubscriber;
            poseStampedSubscriber.TimeStep = 0.1f;
            poseStampedSubscriber.Topic = "/car/" + i + "/pose";
            poseStampedSubscriber.PublishedTransform = car.transform;

            BoxCollider boxCollider = car.AddComponent(typeof(BoxCollider)) as BoxCollider;
            boxCollider.transform.localScale = scale;

            if (i == egoCarNum)
            {
                PoseStampedPublisher poseStampedPublisher = car.AddComponent(typeof(PoseStampedPublisher)) as PoseStampedPublisher;
                poseStampedPublisher.Topic = "/car/" + i + "/nav/goal";

                EgoCarEvaluationInterface evaluationInterface = car.AddComponent(typeof(EgoCarEvaluationInterface)) as EgoCarEvaluationInterface;
                evaluationInterface.SetTargetPosition(initPos);

                LineRenderer lineRenderer = car.AddComponent(typeof(LineRenderer)) as LineRenderer;

                Rigidbody rigidBody = car.AddComponent(typeof(Rigidbody)) as Rigidbody; 
                car.GetComponent<Rigidbody>().isKinematic = true;
            }

            cars.Add(car);
        }
    }

    void Update()
    {
        //TODO - if car is no longer present on track, deactivate car (maybe a timer?)
        for (int i = 0; i < numCars; i++)
        {
            GameObject car = cars[i];

            RosConnector rosConnector = car.GetComponent<RosConnector>();
            bool connected = rosConnector.connectionEstablished ? true : false;
            car.SetActive(connected);

            // update position of car in game
            if (car.activeSelf)
            {
                PoseStampedSubscriber poseStampedSubscriber = car.GetComponent<PoseStampedSubscriber>();
                float x = poseStampedSubscriber.position.x * waypointScaling + 23;
                float z = poseStampedSubscriber.position.z * waypointScaling;
                float y = wheelOffset;
                Vector3 carPos = new Vector3(x, y, z);
                transform.position = carPos;
                transform.rotation = poseStampedSubscriber.rotation;
            }
        }
    }
}
