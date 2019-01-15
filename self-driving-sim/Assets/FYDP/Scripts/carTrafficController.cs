using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityStandardAssets.Vehicles.Car;

[RequireComponent(typeof(CarController))]
public class carTrafficController : MonoBehaviour {
    //TODO:
    //1) Give these cars a way to detect the lanes
    //2) They indicate and change lanes every once in a while
    //3) Make it so they don't drive off the track
    //4) Speed up and slow down every so often, oterwise maintain constant speed
    //5) If in front of ego car, maybe brake lightly
    //6) Speed up and pass other cars
    //7) Don't crash into any other cars
    private CarController m_Car;
    private Steering s;

    private void Awake()
    {
        m_Car = GetComponent<CarController>();
        s = new Steering();
        s.Start();
    }

    //H = steering
    //V = acceleration or foot brake
    // Move(steering, accel, foot brake, handbrake)
    private void FixedUpdate()
    {
        s.UpdateValues();
        m_Car.Move(s.H, s.V, s.V, 0f);
    }
}
