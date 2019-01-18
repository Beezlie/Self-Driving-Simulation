using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//CURRENTLY UNUSED
public class obstacleSpawner : MonoBehaviour
{
    // Obstacle Sprite
    private string[] prefabNames = {"Ambulance", "Audi", "Car", "Mini_truck",
                                "Mini_van", "Police", "taxi", "truck"};
    private int spawnNum;
    private float spawnRate = 3f;    // spawn new obstacle every 3 seconds
    private float maxVertPos = 4.1f;
    private float timer;

    public void Start()
    {
        timer = spawnRate;
    }

    // Update is called once per frame
    private void Update()
    {
        timer -= Time.deltaTime;
        if (timer <= 0)
        {
            // Choose random prefab to spawn
            spawnNum = UnityEngine.Random.Range(0, prefabNames.Length);
            Debug.Log(spawnNum);
            GameObject obstacle = (GameObject)Resources.Load(prefabNames[spawnNum]);

            // Instantiate obstacle at random y position above track
            Vector3 obstaclePos = new Vector3(transform.position.x, UnityEngine.Random.Range(-maxVertPos, maxVertPos), transform.position.z);
            Instantiate(obstacle, obstaclePos, transform.rotation);
            timer = spawnRate;
        }
    }
}