using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CompanionCarManager : MonoBehaviour {

    public int numCompanionCars = 2;        //2 by default
    private GameObject track;
    private Dictionary<GameObject, int> companionCars;      // key = car game object, value = lane
    private List<string> companionCarNames;
    private Dictionary<Vector2, bool> trackOccupancy;

    //Track info
    private float trackLength;
    private float trackWidth;
    private float laneWidth;
    private int numLanes = 6;
    private float[] laneMidPoints;

    private void Start () {
        // Find size of main track
        track = GameObject.Find("Road Piece");
        trackLength = track.gameObject.GetComponent<MeshRenderer>().bounds.size.z;
        trackWidth = track.gameObject.GetComponent<MeshRenderer>().bounds.size.x;

        // Create array of lane midpoints
        laneWidth = trackWidth / numLanes;
        Debug.Log(string.Format("lane width: {0}.", laneWidth));
        laneMidPoints = new float[numLanes];
        for (int i = 0; i < numLanes; i++)
        {
            laneMidPoints[i] = laneWidth * (i + 1);
            Debug.Log(string.Format("lane midpoint: {0}.", laneMidPoints[i]));
        }

        // Find companion car game objects
        companionCars = new Dictionary<GameObject, int>();
        companionCarNames = new List<string>();
        for (int i = 0; i < numCompanionCars; i++)
        {
            string companionCarName = "CompanionCar" + i.ToString();
            GameObject car = GameObject.Find(companionCarName);
            if (car != null)
            {
                companionCarNames.Add(car.name);
                Debug.Log(string.Format("car name: {0}", car.name));

                // Set initial goal as nearest lane midpoint for each car
                int lane = NearestLane(car.transform.position.x);
                companionCars[car] = lane;
            }
            else
            {
                Debug.Log(string.Format("Could not find {0}.", companionCarName));
            }
        }

        // Update companion car goal position every 10 seconds
        InvokeRepeating("UpdateGoal", 0f, 10f);
    }

    private void UpdateGoal()
    {
        foreach (KeyValuePair<GameObject, int> entry in companionCars)
        {
            GameObject car = entry.Key;
            int lane = entry.Value;

            // Select lane and longitudinal goal
            float maxZ = trackLength / 2;
            float goalZ = Mathf.Clamp(Random.Range(car.transform.position.z - maxZ, car.transform.position.z + maxZ), trackLength * 0.1f, trackLength * 0.9f);

            // Only change lanes if not moving backwards
            int newLane;
            if (goalZ >= car.transform.position.z)
            {
                newLane = Mathf.Clamp(Random.Range(lane - 1, lane + 1), 0, numLanes);
            } else
            {
                newLane = lane;
            }

            Vector3 goalPos = new Vector3(laneMidPoints[newLane], 0, goalZ);

            if (!PathOccupied(car, goalPos))
            {
                Debug.Log(string.Format("{0} goal: {1}", car.name, goalPos));
                car.gameObject.GetComponent<CompanionCarInterface>().SetTargetPosition(new Vector3(laneMidPoints[newLane], 0, goalZ));
            }
        }
    }

    // Check if another companion car is occupying path to goal
    private bool PathOccupied(GameObject car, Vector3 goal)
    {
        bool carInPath = false;
        RaycastHit[] hits = Physics.SphereCastAll(car.transform.position, 2f, goal);
        for (int j = 0; j < hits.Length; j++)
        {
            Debug.Log(string.Format("hit names: {0}.", hits[j].transform.gameObject.tag));
            if (companionCarNames.Contains(hits[j].transform.gameObject.tag))
            {
                carInPath = true;
            }
        }
  
        return carInPath;
    }

    // Get the nearest lane to the initial position of the companion car
    private int NearestLane(float x)
    {
        Dictionary<int, float> laneDistances = new Dictionary<int, float>();

        for (int i = 0; i < laneMidPoints.Length; i++)
        {
            laneDistances[i] = Mathf.Abs(x - laneMidPoints[i]);
        }

        int minDiffLane = 0;
        foreach (KeyValuePair<int, float> entry in laneDistances)
        {
            if (entry.Value <= laneDistances[minDiffLane])
            {
                minDiffLane = entry.Key;
            }
        }
        return minDiffLane;
    }
}
