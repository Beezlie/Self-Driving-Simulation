using System.Collections;
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

        float yaw = getYaw(observedPose.rotation.eulerAngles);

        float headingError = getYaw(goal.rotation.eulerAngles) - yaw;
        float headingCorrection = kPHeading * headingError + kDHeading * ((headingError - prevHeadingError) / dt);

        // Incoming pose and velocity are for the rear axles - convert to front.
        Pose frontPose = new Pose(new Vector3(observedPose.position.x + wheelbase + Mathf.Cos(yaw), observedPose.position.y + wheelbase + Mathf.Sin(yaw), 0), observedPose.rotation);
        Vector3 frontLinearVel = new Vector3(linearVel.x + wheelbase * angularVel * Mathf.Sin(yaw), linearVel.y + wheelbase * angularVel * Mathf.Cos(yaw), 0);

        Vector3 intersection = findIntersection(frontPose.position, goal.position, getYaw(goal.rotation.eulerAngles));
        float crosstrackError = Vector3.Distance(intersection, frontPose.position);

        // Goal line always points forward - so, if pose.y is bigger, pose is to the left.
        if (goal.position.y < frontPose.position.y) {
            crosstrackError *= -1;
        }

        float crosstrackCorrection = Mathf.Atan2(kCrosstrack * crosstrackError, velDamping + Vector3.Magnitude(frontLinearVel));

        prevHeadingError = headingError;

        return headingCorrection + crosstrackCorrection; ;
    }

    public void reset()
    {
        prevHeadingError = 0f;
    }
}

