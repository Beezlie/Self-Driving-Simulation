using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;

public class CarPositionUpdater : MonoBehaviour {

    public int numCars = 3;
    public int egoCarNum = 3;

    private GameObject egoCarGoal;
    private List<GameObject> cars;
    private List<GameObject> rosInterfaces;
    private string url = "ws://129.97.69.100:9090";
    private float wheelOffset = 1f;       // offset to apply to y-coord of car so bottom of wheels touch track
    private float waypointScaling = 20f;    //real xMax = 2, yMax = 1, so scale to match virtual track - also in sim y = x and x = y

    // Track if car is no longer visible
    private List<int> numSamePos;
    private List<Vector3> prevPos;

    // TODO: Change init car spawn goal 
    private Vector3 initPosition = new Vector3(0, 0, 0);

    void Start () {

        // Create car objects and subscriber/rosconnector for each car
        cars = new List<GameObject>();
        rosInterfaces = new List<GameObject>();
        numSamePos = new List<int>();
        prevPos = new List<Vector3>();

        for (int i = 0; i < numCars; i++)
        {
            GameObject car = Instantiate(Resources.Load("Prefabs/Car_" + i)) as GameObject;
            car.name = "Car_" + i;
            car.transform.localScale = new Vector3(2.5f, 2.5f, 2.5f);
            car.transform.position = new Vector3(0f, wheelOffset, 0f);
            car.SetActive(false);

            BoxCollider boxCollider = car.AddComponent(typeof(BoxCollider)) as BoxCollider;

            if (egoCarNum == i)
            {
                egoCarGoal = new GameObject();
                egoCarGoal.transform.position = initPosition;

                PoseStampedPublisher poseStampedPublisher = egoCarGoal.AddComponent(typeof(PoseStampedPublisher)) as PoseStampedPublisher;
                poseStampedPublisher.Topic = "/car/" + i + "/nav/goal";
                poseStampedPublisher.PublishedTransform = egoCarGoal.transform;

                EgoCarEvaluationInterface evaluationInterface = car.AddComponent(typeof(EgoCarEvaluationInterface)) as EgoCarEvaluationInterface;
                LineRenderer lineRenderer = car.AddComponent(typeof(LineRenderer)) as LineRenderer;

                Rigidbody rigidBody = car.AddComponent(typeof(Rigidbody)) as Rigidbody; 
                car.GetComponent<Rigidbody>().isKinematic = true;
            }

            car.SetActive(true);
            cars.Add(car);
            prevPos.Add(car.transform.position);
            numSamePos.Add(0);

            GameObject rosInterface = new GameObject();
            rosInterface.name = "RosInterface" + i;
            rosInterface.transform.position = new Vector3(0f, 0f, 0f);
            rosInterface.SetActive(false);

            RosConnector rosConnector = rosInterface.AddComponent(typeof(RosConnector)) as RosConnector;
            rosConnector.Protocol = RosConnector.Protocols.WebSocketSharp;
            rosConnector.Timeout = 10;
            rosConnector.RosBridgeServerUrl = url;
            rosConnector.Awake();

            PoseStampedSubscriber poseStampedSubscriber = rosInterface.AddComponent(typeof(PoseStampedSubscriber)) as PoseStampedSubscriber;
            poseStampedSubscriber.TimeStep = 0.1f;
            poseStampedSubscriber.Topic = "/car/" + i + "/pose";
            poseStampedSubscriber.PublishedTransform = rosInterface.transform;
            poseStampedSubscriber.Awake();

            rosInterface.SetActive(true);
            rosInterfaces.Add(rosInterface);
        }
    }

    void Update()
    {
        for (int i = 0; i < numCars; i++)
        {
            GameObject car = cars[i];
            GameObject rosInterface = rosInterfaces[i];

            RosConnector rosConnector = rosInterface.GetComponent<RosConnector>();
            bool connected = rosConnector.connectionEstablished ? true : false;

            PoseStampedSubscriber poseStampedSubscriber = rosInterface.GetComponent<PoseStampedSubscriber>();
            float x = poseStampedSubscriber.position.x * waypointScaling + 24;
            float z = poseStampedSubscriber.position.z * waypointScaling;
            float y = wheelOffset;
            Vector3 carPos = new Vector3(x, y, z);
            car.transform.position = carPos;
            car.transform.rotation = poseStampedSubscriber.rotation;
            Debug.Log(string.Format("Car {0} pos: {1}", i, carPos));

            numSamePos[i] = prevPos[i] == poseStampedSubscriber.position ? numSamePos[i] + 1 : 0;
            prevPos[i] = poseStampedSubscriber.position;

            bool isCarActive = numSamePos[i] > 50 ? false : true;
            car.SetActive(isCarActive);
            Debug.Log(string.Format("Car {0} numSamePos: {1}", i, numSamePos[i]));
            Debug.Log(string.Format("Car {0} pos: {1}", i, poseStampedSubscriber.position));
            Debug.Log(string.Format("Car {0} prevPos: {1}", i, prevPos[i]));
        }
    }
}
