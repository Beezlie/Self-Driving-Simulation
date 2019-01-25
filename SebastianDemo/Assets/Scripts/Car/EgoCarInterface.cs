using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace PathfindingForCars
{
    public class EgoCarInterface : MonoBehaviour
    {
        private Vector3 targetPosition;
        private Quaternion targetRotation;
        private float targetHeading;
        private float targetSpeed;

        //Reference to the car data belonging to this car
        private CarData carData;

        // Map coordinates
        private int mapLength = PathfindingController.mapLength;
        private int mapWidth = PathfindingController.mapWidth;

        // Use this for initialization
        void Start()
        {
            carData = GetComponent<CarData>();
            targetPosition = new Vector3(180, 0, 18);
        }

        // Update is called once per frame
        void Update()
        {

        }

        public Transform GetCurrentTransform()
        {
            return carData.GetCarTransform();
        }

        public Vector3 GetCurrentPosition()
        {
            return carData.GetRearWheelPos();
        }

        public float GetCurrentHeading()
        {
            return carData.GetHeading();
        }

        private float GetCurrentSpeed()
        {
            return carData.GetSpeed();
        }

        public void SetTargetPosition(Vector3 position)
        {
            targetPosition = position;
        }

        public void SetTargetHeading(float heading)
        {
            targetHeading = heading;
        }

        private void SetTargetSpeed(float speed)
        {
            targetSpeed = speed;
        }

        public Vector3 GetTargetPosition()
        {
            return targetPosition;
        }

        public float GetTargetHeading()
        {
            return targetHeading;
        }

        private float GetTargetSpeed()
        {
            return targetSpeed;
        }
    }
}
