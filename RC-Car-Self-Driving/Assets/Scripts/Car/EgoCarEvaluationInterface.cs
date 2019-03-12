using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;


public class EgoCarEvaluationInterface : MonoBehaviour {

    // Environment info
    private GameObject trackData;
    private GameObject road;

    // Car info
    private PoseStampedPublisher goalPublisher;
    private Vector3 goalPos;

    // Car detection parameters
    private int numLineSegments = 100;
    private float detectionRadius = 7.5f;
    LineRenderer line;

    public float GetTrackWidth()
    {
        return road.gameObject.GetComponent<MeshRenderer>().bounds.size.x;
    }

    public float GetTrackLength()
    {
        return road.gameObject.GetComponent<MeshRenderer>().bounds.size.z;
    }

    public float GetTrackVelocity()
    {
        return trackData.gameObject.GetComponent<TrackSpeedSubscriber>().GetVelocity();
    }

    public Transform GetCurrentTransform()
    {
        return transform;
    }

    public Vector3 GetCurrentPosition()
    {
        return transform.position;
    }

    //TODO - implement this somehow
    /*
    private float GetCurrentSpeed()
    {
        return carState.dz;
    }
    */

    public void SetTargetPosition(Vector3 position)
    {
        goalPos = position;
    }

    public Vector3 GetTargetPosition()
    {
        return goalPos;
    }

    //Search through all obstacles to find which fall within a detection radius of the car
    public List<Vector3> DetectObstaclesWithinRadiusOfCar()
    {
        List<Vector3> obstaclePos = new List<Vector3>();
        Collider[] obstacles = Physics.OverlapSphere(transform.position, detectionRadius);
        foreach (Collider collider in obstacles)
        {
            if (collider is BoxCollider && collider.transform.position != transform.position)
            {
                obstaclePos.Add(collider.transform.position);
                Debug.Log(string.Format("obstacle position: {0}", collider.transform.position));
            }
        }
        return obstaclePos;
    }

    private void Awake()
    {
        // Find track object so velocity can be fed to car controller
        trackData = GameObject.Find("TrackData");
        if (trackData == null)
        {
            Debug.Log("The trackData object was not found.");
        }
        road = GameObject.Find("Road Piece");
        if (road == null)
        {
            Debug.Log("The Road Piece object was not found.");
        }
    }

    private void Start()
    {
        // Set initial gosl
        SetTargetPosition(transform.position);

        /*
        // Get goal publisher attached to ego car
        goalPublisher = gameObject.GetComponent(typeof(PoseStampedPublisher)) as PoseStampedPublisher;
        */

        //Draw the lidar visualization
        line = gameObject.GetComponent<LineRenderer>();
        line.SetVertexCount(numLineSegments + 1);
        line.useWorldSpace = false;
        DrawLidar();
    }

    private void Update()
    {
        /*
        //Publish the EgoCar's goal pose
        goalPublisher.PublishedTransform.position = goalPos;
        goalPublisher.PublishedTransform.rotation = transform.rotation;
        */
    }

    // Draw circle around car representing detection range of Lidar sensor
    private void DrawLidar()
    {
        float x;
        float y;
        float z;

        float angle = 20f;

        for (int i = 0; i < (numLineSegments + 1); i++)
        {
            x = Mathf.Sin(Mathf.Deg2Rad * angle) * detectionRadius;
            z = Mathf.Cos(Mathf.Deg2Rad * angle) * detectionRadius;

            line.SetPosition(i, new Vector3(x, 0, z));

            angle += (360f / numLineSegments);
        }
    }
}
