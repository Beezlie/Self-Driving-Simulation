using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;

namespace PathfindingForCars
{
    //Takes care of all pathfinding
    public class PathfindingController : MonoBehaviour
    {
        //Drags
        //The target we want to reach
        public Transform targetCarTrans;
        //The marker to mark the end of the path and the rotation the car should have
        public Transform targetMarkerTrans;

        //Map data
        //The size of all cells in [m]
        public static int mapLength = 200;
        public static int mapWidth = 50;
        //The size of one cell in [m]
        public static float cellWidth = 1f;

        //External scripts
        private DebugController debugController;

        //Objects
        private HybridAStar hybridAStar;
        private HybridAStarAngle hybridAStarAngle;
        private HeuristicsController heuristicsController;
        private SmoothPathController smoothPathController;



        void Awake()
        {
            heuristicsController = new HeuristicsController();

            smoothPathController = new SmoothPathController();

            debugController = GetComponent<DebugController>();
        }



        void Start()
        {
            //Generate obstacles
            //Has to do it from this script or the obstacles might be created after this script has
            //finished and mess things up
            GetComponent<ObstaclesController>().InitObstacles();

            //Hide the marker showing where the car should drive
            targetMarkerTrans.gameObject.SetActive(false);

            hybridAStar = new HybridAStar();
            hybridAStarAngle = new HybridAStarAngle();


            //Create the textures showing various stuff
            debugController.DisplayCellObstacleIntersection();
            debugController.DisplayObstacleFlowField();
        }



        void Update()
        {
            //Try to find a path if we press left mouse
            if (Input.GetMouseButtonDown(0))
            {
                //Check if the target car has a valid position
                if (HasTargetCarValidPosition(targetCarTrans.GetComponent<CarData>()))
                {
                    //Stop the car
                    SimController.current.StopCar();

                    //Wait for the car to stop before trying to find a path
                    StartCoroutine(WaitForCarToStop());
                }
            }
        }



        //Wait for the car to stop before we generate a new path 
        //or it might have passed the start position of the path when the path is finished
        IEnumerator WaitForCarToStop()
        {
            while (SimController.current.GetActiveCarData().GetSpeed() > 5f)
            {
                yield return null;
            }

            //Now we need to check again if the target position is possible
            //because we might have moved the target while the car was braking        
            if (HasTargetCarValidPosition(targetCarTrans.GetComponent<CarData>()))
            {
                //The car has stopped and the target should be possible, so generate a path
                StartCoroutine(GeneratePath());

                //Move the marker car to the end of the path
                targetMarkerTrans.position = targetCarTrans.position;
                targetMarkerTrans.rotation = targetCarTrans.rotation;

                targetMarkerTrans.gameObject.SetActive(true);
            }
        }



        //Generate a path and send it to the car
        IEnumerator GeneratePath()
        {
            IntVector2 targetCellPos = ConvertCoordinateToCellPos(targetCarTrans.position);

            //To measure time, is measured in tick counts so no float
            int startTime = 0;


            //
            //Calculate Heuristics
            //
            //Calculate euclidean distance heuristic
            startTime = Environment.TickCount;

            heuristicsController.EuclideanDistance(targetCellPos);

            DisplayTime(startTime, Environment.TickCount, "Euclidean Distance");

            yield return new WaitForSeconds(0.05f);


            //Calculate dynamic programing
            startTime = Environment.TickCount;

            heuristicsController.DynamicProgramming(targetCellPos);

            DisplayTime(startTime, Environment.TickCount, "Dynamic Programming");

            yield return new WaitForSeconds(0.05f);

            //Calculate the final heuristics
            heuristicsController.GenerateFinalHeuristics();


            //
            //Update A*
            //
            //First we have to check if a path is even possible by using the flow field
            //If the cell the car starts in has a distance which is the max distance, then a path is not possible
            IntVector2 carCellPos = ConvertCoordinateToCellPos(SimController.current.GetActiveCarData().GetRearWheelPos());

            if ((HeuristicsController.flowFieldHeuristics[carCellPos.x, carCellPos.z] <= (float)System.Int32.MaxValue))
            {
                List<Node> finalPath = new List<Node>();
                //List with all expanded nodes for debugging
                List<Node> expandedNodes = new List<Node>();

                startTime = Environment.TickCount;

                //The output from this is finalPath and expandedNodes
                //hybridAStar.GenerateHybridAStarPath(targetCarTrans, finalPath, expandedNodes);

                //The slower but more accurate version
                hybridAStarAngle.GenerateHybridAStarPath(targetCarTrans, finalPath, expandedNodes);

                DisplayTime(startTime, Environment.TickCount, "Hybrid A Star");


                //Reset debugger
                debugController.Reset();


                //Always display the search tree even if we havent found a path to the goal
                debugController.DisplaySearchTree(expandedNodes);



                //
                //Smooth the path and send it to the car
                //
                //If we have found a path
                if (finalPath.Count > 0)
                {
                    //Smooth the path
                    startTime = Environment.TickCount;

                    List<Node> smoothPath = smoothPathController.GetSmoothPath(finalPath, false);

                    DisplayTime(startTime, Environment.TickCount, "Smooth path");

                    SimController.current.SendPathToActiveCar(smoothPath);

                    //Debug if the smooth path is working
                    debugController.DisplayFinalPath(finalPath, smoothPath);
                }
            }



            yield return null;
        }



        //Check if the target car has a valid position
        private bool HasTargetCarValidPosition(CarData carData)
        {
            bool hasValidPosition = false;

            Vector3 carPos = carData.GetRearWheelPos();

            float targetCarHeading = carData.GetHeading() * Mathf.Deg2Rad;

            if (!ObstaclesDetection.HasCarInvalidPosition(carPos, targetCarHeading, carData))
            {
                hasValidPosition = true;
            }

            return hasValidPosition;
        }



        //Convert from world position to a cell pos
        public static IntVector2 ConvertCoordinateToCellPos(Vector3 coordinate)
        {
            IntVector2 arrayPos = new IntVector2();

            arrayPos.x = Mathf.FloorToInt(coordinate.x / cellWidth);
            arrayPos.z = Mathf.FloorToInt(coordinate.z / cellWidth);

            return arrayPos;
        }



        //Is a cell position within the grid?
        public static bool IsCellWithinGrid(IntVector2 cellPos)
        {
            bool isWithIn = false;

            if (cellPos.x >= 0 && cellPos.x < mapLength && cellPos.z >= 0 && cellPos.z < mapWidth)
            {
                isWithIn = true;
            }

            return isWithIn;
        }



        //Is a world position within the grid?
        public static bool IsPositionWithinGrid(Vector3 worldPos)
        {
            bool isWithIn = false;

            if (worldPos.x > 0f && worldPos.x < mapLength && worldPos.z > 0f && worldPos.z < mapWidth)
            {
                isWithIn = true;
            }

            return isWithIn;
        }



        //Help function to display how long something took
        private void DisplayTime(int startTime, int endTime, string text)
        {
            float timeInSeconds = (endTime - startTime) / 1000f;

            Debug.Log(text + " took " + timeInSeconds + " seconds");
        }
    }
}
