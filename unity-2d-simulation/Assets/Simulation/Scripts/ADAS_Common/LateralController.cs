using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// references https://bitbucket.org/dshin-uwaterloo/adas-car-on-treadmill-v2/src/master/src/adas_common/src/lateral_controllers.cpp
public class LateralController
{
    protected bool goalReceived;
    protected Pose goal;

    public LateralController()
    {
        goalReceived = false;
    }

    public void setGoal(Pose g)
    {
        goalReceived = true;
        goal = g;
    }

    public Pose getGoal()
    {
        return goal;
    }

    protected Vector3 findIntersection(Vector3 point, Vector3 linePoint, float lineAngle)
    {
        return findIntersection(point, linePoint, new Vector3(1, Mathf.Tan(lineAngle), 0));
    }

    protected Vector3 findIntersection(Vector3 point, Vector3 linePoint, Vector3 lineSlope)
    {
        Vector3 pointSlope = new Vector3(lineSlope.y, -lineSlope.x, 0);

        /*
            * Two parameteric equations:
            * L0 = lineSlope * t + linePoint
            * L1 = pointSlope * u + point
            * Solve for t.
            */
        float intersectT = (pointSlope.y * (point.x - linePoint.x) -
                            pointSlope.x * (point.y - linePoint.y)) /
                            (lineSlope.x * pointSlope.y - pointSlope.x * lineSlope.y);

        return lineSlope * intersectT + linePoint;
    }

    //TODO - check that this function actually works
    protected float getYaw(Vector3 eulerAngles)
    {
        float yaw = eulerAngles.z * Mathf.Deg2Rad;
        if (yaw > Mathf.PI)
        {
            yaw = 2 * Mathf.PI - yaw;
        }
        return yaw;
    }
}