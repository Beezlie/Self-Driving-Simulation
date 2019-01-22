using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PurePursuitController : LateralController
{
    private float lookahead;
    private float axleDistance;

    public PurePursuitController(float lookahead, float axleDistance)
    {
        this.lookahead = lookahead;
        this.axleDistance = axleDistance;

        verifyLookahead();
        if (axleDistance <= 0) {
            throw new System.ArgumentException("Axle distance must be positive.");
        }
    }

    private void verifyLookahead()
    {
        if (lookahead <= 0) {
            throw new System.ArgumentException("Lookahead must be positive.");
        }
    }

    public void setLookahead(float l)
    {
        lookahead = l;
        verifyLookahead();
    }

    // Given a straight trajectory, and the current pose, calculate the steering value
    public float commandStep(Pose observedPose)
    {
        if (!goalReceived) {
            return 0;
        }

        Vector3 goalSlope = new Vector3(1, Mathf.Tan(getYaw(goal.rotation)));
        Vector3 intersection = findIntersection(observedPose.position, goal.position, goalSlope);

        float intersectionDistance2 = Vector3.Distance(observedPose.position, observedPose.position);

        float lookahead2 = Mathf.Pow(lookahead, 2);

        Vector3 lookaheadPoint;

        // Direction unit vector of the goal trajectory.
        Vector3 trajectoryDir = Vector3.Normalize(goalSlope);
        if (intersectionDistance2 >= lookahead2) {
            /*
                * Distance to the intersection is larger than the lookahead distance.
                * Simply look ahead a fixed amount.
                */
            lookaheadPoint = intersection + lookahead * trajectoryDir;
        } else {
            /*
                * Lookahead distance (Forward from the intersection) decreases the further
                * you are away from the trajectory.
                */
            lookaheadPoint = intersection + Mathf.Sqrt(lookahead2 - intersectionDistance2) * trajectoryDir;
        }

        Vector3 poseToTarget = lookaheadPoint - observedPose.position;
        float goalYaw = Vector3.Angle(new Vector3(1, 0, 0), poseToTarget);
        // Vector3.Angle() always returns positive - give the directional context.
        if (poseToTarget.y < 0) {
            goalYaw *= -1;
        }

        float poseYaw = getYaw(observedPose.rotation);

        // Now use the pure pursuit equation to calculate the desired steering angle.
        float goalCurvature = 2f * Mathf.Sin(goalYaw - poseYaw) / lookahead;
        return Mathf.Atan(goalCurvature * axleDistance);
    }
}
