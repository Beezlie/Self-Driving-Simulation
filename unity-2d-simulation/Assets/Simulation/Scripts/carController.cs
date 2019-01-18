using UnityEngine;

// Move car sprite to waypoints
public class carController : MonoBehaviour
{
    private GameObject car;
    private Vector3 waypoint;
    private float carSpeed = 5f;

    void Update()
    {
        // Receive waypoint for car to navigate to from ROS server
        waypoint = new Vector3(0, 0, 0);
   
        // The step size is equal to speed times frame time
        float step = carSpeed * Time.deltaTime;

        // Move car position a step closer to the target.
        transform.position = Vector3.MoveTowards(transform.position, waypoint, step);
    }


    void OnCollisionEnter2D(Collision2D col)
    {
        if (col.gameObject.tag == "Enemy Car")
        {
            //Destroy (gameObject);
            gameObject.SetActive(false);
        }
    }
}