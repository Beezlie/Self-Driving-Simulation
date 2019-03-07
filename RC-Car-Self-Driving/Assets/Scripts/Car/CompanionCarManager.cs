using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CompanionCarManager : MonoBehaviour {

    public int numCompanionCars = 2;        //2 by default
    private List<GameObject> companionCars;

    private void Start () {
        InvokeRepeating("UpdateGoal", 0f, 10f);

        // Find companion car game objects
        companionCars = new List<GameObject>();
        for (int i = 0; i < numCompanionCars; i++)
        {
            string companionCarName = "CompanionCar" + i.ToString();
            GameObject car = GameObject.Find(companionCarName);
            if (car != null)
            {
                companionCars.Add(car);
            }
            else
            {
                Debug.Log(string.Format("Could not find {0}.", companionCarName));
            }
        }
    }

    private void UpdateGoal()
    {
        foreach (GameObject car in companionCars)
        {
            float goalZ = Mathf.Clamp(Random.Range(car.transform.position.z - 20, car.transform.position.z + 20), 5, 35);
            car.gameObject.GetComponent<CompanionCarInterface>().SetTargetPosition(new Vector3(car.transform.position.x, 0, goalZ));
        }
    }
}
