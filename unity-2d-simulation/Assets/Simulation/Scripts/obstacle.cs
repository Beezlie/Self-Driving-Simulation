using System;
using UnityEngine;

public class obstacle : MonoBehaviour {
	private float speed = 5f;
    private string obstacleID;
    private float elapsed = 0f;
	
	// Update is called once per frame
	void Update () {
		transform.Translate (new Vector3 (0,1,0) * speed * Time.deltaTime);
	}

	void OnBecameInvisible() {
		Destroy(gameObject);
	}
}
