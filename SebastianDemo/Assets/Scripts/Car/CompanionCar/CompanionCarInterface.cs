using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace PathfindingForCars
{
    public class CompanionCarInterface : MonoBehaviour
    {
        private Vector3 targetPosition;
        private float targetHeading;
        private float targetSpeed;

        //Reference to the car data belonging to this car
        private CompanionCarData compCarData;

        // Map coordinates
        private int mapLength = PathfindingController.mapLength;
        private int mapWidth = PathfindingController.mapWidth;

        // Lidar detection parameters
        private int numLineSegments = 100;
        private float detectionRadius = 20f;
        LineRenderer line;

        // Use this for initialization
        void Start()
        {
            compCarData = GetComponent<CompanionCarData>();
            targetPosition = new Vector3(105, 0, 40);

            //Draw the lidar visualization
            line = gameObject.GetComponent<LineRenderer>();
            line.SetVertexCount(numLineSegments + 1);
            line.useWorldSpace = false;
            DrawLidar();
        }

        // Update is called once per frame
        void Update()
        {
            // Reset car if close to end of track
            if (GetCurrentPosition().x >= (mapLength - 10))  
            {
                ResetEgoPosition();
            }

            // Detect obstacles surrounding the vehicle
            List<ObstacleData> obstacles = DetectObstaclesWithinRadiusOfCar();
            if (obstacles.Count > 0)
            {
                for (int i = 0; i < obstacles.Count; i++)
                {
                    ObstacleData obstacle = obstacles[i];
                    //Debug.Log(string.Format("Obstacle detected at position: {0}", obstacle.centerPos));
                }
            }
        }

        public Transform GetCurrentTransform()
        {
            return compCarData.GetCarTransform();
        }

        public Vector3 GetCurrentPosition()
        {
            return compCarData.GetRearWheelPos();
        }

        public float GetCurrentHeading()
        {
            return compCarData.GetHeading();
        }

        public float GetCurrentSpeed()
        {
            return compCarData.GetSpeed();
        }

        public void SetTargetPosition(Vector3 position)
        {
            targetPosition = position;
        }

        public void SetTargetHeading(float heading)
        {
            targetHeading = heading;
        }

        public void SetTargetSpeed(float speed)
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

        public float GetTargetSpeed()
        {
            return targetSpeed;
        }

        private void ResetEgoPosition()
        {
            transform.position = new Vector3(10, 0, 5);        // TODO: use mapwidth 
            transform.eulerAngles = new Vector3(0, 90, 0);

            //Reset target position
            targetPosition = transform.position;
        }

        private void OnCollisionEnter(Collision collision)
        {
            Debug.Log("Collision detected");
            ResetEgoPosition();
        }

        //Search through all obstacles to find which fall within a radius of the car
        public List<ObstacleData> DetectObstaclesWithinRadiusOfCar()
        {
            //The list with close obstacles
            List<ObstacleData> closeObstacles = new List<ObstacleData>();

            //The list with all obstacles in the map
            List<ObstacleData> allObstacles = ObstaclesController.obstaclesPosList;

            //Find close obstacles
            for (int i = 0; i < allObstacles.Count; i++)
            {
                float distSqr = (GetCurrentPosition() - allObstacles[i].centerPos).sqrMagnitude;

                //Add to the list of close obstacles if close enough
                if (distSqr < detectionRadius * detectionRadius)
                {
                    closeObstacles.Add(allObstacles[i]);
                }
            }

            return closeObstacles;
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
}
