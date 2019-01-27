using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace PathfindingForCars
{
    //Will make it easier if we have several self-driving cars than to contact the self-driving car directly
    public class CompanionSimController : MonoBehaviour
    {
        public static CompanionSimController current;

        //The self-driving car
        public Transform selfDrivingCar;

        private CompanionCarController compCarController;



        void Start()
        {
            current = this;

            compCarController = selfDrivingCar.GetComponent<CompanionCarController>();
        }



        void Update()
        {

        }



        //
        // Set and get
        //
        public void SendPathToActiveCar(List<Node> wayPoints)
        {
            Debug.Log("Sending waypoints to companion car." + wayPoints);
            compCarController.SendPathToCar(wayPoints);
        }

        //Get data such as speed, length, etc
        public CompanionCarData GetActiveCarData()
        {
            return compCarController.GetCarData();
        }

        //Stop the active car from driving
        public void StopCar()
        {
            SendPathToActiveCar(null);

            compCarController.StopCar();
        }
    }
}
