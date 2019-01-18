using UnityEngine;
using System.Collections;

public class CarSpawner : MonoBehaviour {

	public GameObject cars;
	int carNo;
	public float maxPos = 2.2f;
	public float delayTimer = 0.5f;

	float timer;

	// Use this for initialization
	void Start () {
		timer = delayTimer;
	}
	
	// Update is called once per frame
	void Update () {
        
		timer -= Time.deltaTime;

		if (timer <= 0) {
            Vector3 carPos = new Vector3(transform.position.x, Random.Range(-maxPos, maxPos),transform.position.z);
			carNo = Random.Range (0,5);
			Instantiate (cars, carPos, transform.rotation);
			timer = delayTimer;
		}
	}
}
