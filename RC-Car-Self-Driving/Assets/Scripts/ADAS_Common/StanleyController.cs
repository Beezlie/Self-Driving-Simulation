﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//references https://bitbucket.org/dshin-uwaterloo/adas-car-on-treadmill-v2/src/master/src/adas_common/src/lateral_controllers.cpp
public class StanleyController : LateralController
{
    private float kPHeading;
    private float kDHeading;
    private float kCrosstrack;
    private float velDamping;
    private float wheelbase;
    private float prevHeadingError;

    public StanleyController(float kPHeading, float kDHeading, float kCrosstrack, float velDamping, float wheelbase)
    {
        this.kPHeading = kPHeading;
        this.kDHeading = kDHeading;
        this.kCrosstrack = kCrosstrack;
        this.velDamping = velDamping;
        this.wheelbase = wheelbase;
        prevHeadingError = 0f;

        if (wheelbase <= 0)  {
            throw new System.ArgumentException("Wheelbase must be larger than zero.");
        }
        verifyControlParams();
    }

    private void verifyControlParams()
    {
        if (kPHeading < 0) {
            throw new System.ArgumentException("P gain for heading error cannot be negative.");
        }
        if (kDHeading < 0) {
            throw new System.ArgumentException("D gain for heading error cannot be negative.");
        }
        if (kCrosstrack < 0) {
            throw new System.ArgumentException("Crosstrack gain cannot be negative.");
        }
        if (velDamping < 0) {
            throw new System.ArgumentException("WVelocity damping constant cannot be negative.");
        }
    }

    public void setKPHeading(float kP)
    {
        kPHeading = kP;
        verifyControlParams();
    }

    public void setKDHeading(float kD)
    {
        kDHeading = kD;
        verifyControlParams();
    }

    public void setKCrosstrack(float kCross)
    {
        kCrosstrack = kCross;
        verifyControlParams();
    }

    public void setVelDamping(float vDamping)
    {
        velDamping = vDamping;
        verifyControlParams();
    }

    public float commandStep(Pose observedPose, Vector3 linearVel, float angularVel, float dt)
    {
        if (!goalReceived) {
            return 0f;
        }

        Debug.Log(string.Format("observedPose: {0}", observedPose));

        float yaw = getYaw(observedPose.rotation.eulerAngles);
        Debug.Log(string.Format("yaw in command step: {0}", yaw));

        float headingError = getYaw(goal.rotation.eulerAngles) - yaw;
        Debug.Log(string.Format("headingError: {0}", headingError));

        float headingCorrection = kPHeading * headingError + kDHeading * ((headingError - prevHeadingError) / dt);
        Debug.Log(string.Format("headingCorrection: {0}", headingCorrection));

        // Incoming pose and velocity - convert to front axle.
        Pose frontPose = new Pose(new Vector3(observedPose.position.x + wheelbase * Mathf.Cos(yaw), observedPose.position.y + wheelbase * Mathf.Sin(yaw), 0), observedPose.rotation);
        Debug.Log(string.Format("frontPose: {0}", frontPose));

        Vector3 frontLinearVel = new Vector3(linearVel.x + angularVel * Mathf.Sin(yaw), linearVel.y + angularVel * Mathf.Cos(yaw), 0);
        Debug.Log(string.Format("frontLinearVel: {0}", frontLinearVel));

        Vector3 intersection = findIntersection(frontPose.position, goal.position, getYaw(goal.rotation.eulerAngles));
        Debug.Log(string.Format("intersection: {0}", intersection));

        float crosstrackError = Vector3.Distance(intersection, frontPose.position);
        Debug.Log(string.Format("cross track error: {0}", crosstrackError));

        // Goal line always points forward - so, if pose.y is bigger, pose is to the left.
        if (goal.position.y < frontPose.position.y) {
            crosstrackError *= -1;
        }

        float crosstrackCorrection = Mathf.Atan2(kCrosstrack * crosstrackError, velDamping + Vector3.Magnitude(frontLinearVel));
        Debug.Log(string.Format("cross track correction: {0}", crosstrackCorrection));

        prevHeadingError = headingError;

        cte.Add(crosstrackError.ToString());
        time.Add(Time.time.ToString());
        WriteToFile();

        return headingCorrection + crosstrackCorrection; ;
    }

    public void reset()
    {
        prevHeadingError = 0f;
    }

    List<string> cte = new List<string>();
    List<string> time = new List<string>();
    public void WriteToFile()
    {
        System.IO.File.WriteAllLines(@"C:\Users\Angelo\FYDP\cte_data.txt", cte);
        System.IO.File.WriteAllLines(@"C:\Users\Angelo\FYDP\time_data.txt", time);
    }
}

