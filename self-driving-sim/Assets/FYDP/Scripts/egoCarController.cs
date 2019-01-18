﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityStandardAssets.Vehicles.Car;

[RequireComponent(typeof(CarController))]
public class egoCarController : MonoBehaviour
{
    //TODO - control the steering/acceleration/braking of this car using reinforcement learning model
    //Maybe the RL model just tells the car where it should go and this handles actually getting it there
    //Check if there is already code to send the car to a waypoint
    private CarController m_Car;

    public float SteeringAngle { get; set; }
    public float Acceleration { get; set; }
    private Steering s;
   // private Collider collider;

    private void Awake()
    {
        // get the car controller
        m_Car = GetComponent<CarController>();
        s = new Steering();
        //collider = GetComponent<Collider>();
        s.Start();
    }

    private void FixedUpdate()
    {
        // If holding down W or S control the car manually
        if (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.S))
        {
            s.UpdateValues();
            m_Car.Move(s.H, s.V, s.V, 0f);
        }
        else
        {
            m_Car.Move(SteeringAngle, Acceleration, Acceleration, 0f);
        }
    }

    /*
    private void Update()
    {
        // get the distance to ground
        float distToGround = collider.bounds.extents.y;
        if (!IsGrounded(distToGround)) {
            ResetEgoPosition();
        }

    }

    private bool IsGrounded(float distToGround)
    {
        return Physics.Raycast(transform.position, -Vector3.up, distToGround + 0.1f);
    }

    private void ResetEgoPosition()
    {
        this.transform.position = new Vector3(0, 0, 0);
        this.transform.rotation = new Quaternion(0, 0, 0, 0);
    }
    */
}