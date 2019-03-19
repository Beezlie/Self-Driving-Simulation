using UnityEngine;
using RosSharp.RosBridgeClient;

public class GoalPositionPublisher : MonoBehaviour
{
    private PoseStampedPublisher goalPublisher;
    private GameObject egoCar;

    private void Awake()
    {
        egoCar = GameObject.Find("EgoCar");
        if (egoCar == null)
        {
            Debug.Log("The EgoCar object was not found.");
        }
    }

    private void Start()
    {
        // Initialize pose publisher
        goalPublisher = gameObject.GetComponent(typeof(PoseStampedPublisher)) as PoseStampedPublisher;
    }

    private void Update()
    {
        //Publish the EgoCar's goal pose
        goalPublisher.PublishedTransform.position = egoCar.GetComponent<EgoCarTrainingInterface>().GetTargetPosition();
        goalPublisher.PublishedTransform.rotation = egoCar.transform.rotation;
    }
}


