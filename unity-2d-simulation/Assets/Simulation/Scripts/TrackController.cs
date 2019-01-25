using UnityEngine;

public class TrackController : MonoBehaviour
{
    public float speed;
    private float elapsed = 0f;
    Vector2 offset;

    private void Update()
    {
        //move the track in the x direction
        offset = new Vector2(0, Time.time * speed);
        GetComponent<Renderer>().material.mainTextureOffset = offset;
    }
}
