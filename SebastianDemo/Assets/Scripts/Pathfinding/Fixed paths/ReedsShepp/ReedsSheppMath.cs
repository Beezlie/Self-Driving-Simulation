using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using PathfindingForCars;

namespace FixedPathAlgorithms
{
    public class ReedsSheppMath
    {
        //The radius the car can turn with
        private float turningRadius;


        public float TurningRadius
        {
            get
            {
                return this.turningRadius;
            }
            set
            {
                this.turningRadius = value;
            }
        }



        public ReedsSheppMath(float turningRadius)
        {
            this.turningRadius = turningRadius;
        }



        //
        // Calculate center positions of the left and right circle
        //

        //Right circle
        public Vector3 GetRightCircleCenterPos(Vector3 carPos, float heading)
        {
            Vector3 rightCirclePos = Vector3.zero;

            //90 degrees to the right of the car
            rightCirclePos.x = carPos.x + turningRadius * Mathf.Sin(heading + (Mathf.PI / 2f));

            rightCirclePos.z = carPos.z + turningRadius * Mathf.Cos(heading + (Mathf.PI / 2f));

            return rightCirclePos;
        }

        //Left circle
        public Vector3 GetLeftCircleCenterPos(Vector3 carPos, float heading)
        {
            Vector3 rightCirclePos = Vector3.zero;

            //90 degrees to the left of the car
            rightCirclePos.x = carPos.x + turningRadius * Mathf.Sin(heading - (Mathf.PI / 2f));

            rightCirclePos.z = carPos.z + turningRadius * Mathf.Cos(heading - (Mathf.PI / 2f));

            return rightCirclePos;
        }



        //
        // Calculate the start and end positions of the tangent lines
        //

        //Get the CCC tangent points
        public void Get_CCC_Tangents(
            Vector3 startCircle,
            Vector3 goalCircle,
            bool isRLR,
            OneReedsSheppPath pathData)
        {
            //The distance between the circles
            float D = (startCircle - goalCircle).magnitude;

            //The angle between the goal and the new circle we create
            float theta = Mathf.Acos(D / (4f * turningRadius));

            //But we need to modify the angle theta if the circles are not on the same line
            Vector3 V1 = goalCircle - startCircle;

            //Different depending on if we calculate LRL or RLR
            if (!isRLR)
            {
                theta = Mathf.Atan2(V1.z, V1.x) + theta;
            }
            else
            {
                theta = Mathf.Atan2(V1.z, V1.x) - theta;
            }


            //Calculate the position of the third circle
            float x = startCircle.x + 2f * turningRadius * Mathf.Cos(theta);
            float y = startCircle.y;
            float z = startCircle.z + 2f * turningRadius * Mathf.Sin(theta);

            Vector3 middleCircleCenter = new Vector3(x, y, z);


            //Calculate the tangent points
            Vector3 V2 = (startCircle - middleCircleCenter).normalized;
            Vector3 V3 = (goalCircle - middleCircleCenter).normalized;

            Vector3 startTangent = middleCircleCenter + V2 * turningRadius;
            Vector3 goalTangent = middleCircleCenter + V3 * turningRadius;


            //Save everything
            pathData.middleCircleCoordinate = middleCircleCenter;

            pathData.startTangent = startTangent;
            pathData.goalTangent = goalTangent;
        }



        //Outer tangent (LSL and RSR)
        public void LSLorRSR(
            Vector3 startCircle,
            Vector3 goalCircle,
            bool isBottom,
            OneReedsSheppPath pathData)
        {
            //The angle to the first tangent coordinate is always 90 degrees if the both circles have the same radius
            float theta = 90f * Mathf.Deg2Rad;

            //Need to modify theta if the circles are not on the same height (z)
            theta += Mathf.Atan2(goalCircle.z - startCircle.z, goalCircle.x - startCircle.x);

            //Add pi to get the "bottom" coordinate which is on the opposite side (180 degrees = pi)
            if (isBottom)
            {
                theta += Mathf.PI;
            }

            //The coordinates of the first tangent points
            float xT1 = startCircle.x + turningRadius * Mathf.Cos(theta);
            float zT1 = startCircle.z + turningRadius * Mathf.Sin(theta);

            //To get the second coordinate we need a direction
            //This direction is the same as the direction between the center pos of the circles
            Vector3 dirVec = goalCircle - startCircle;

            float xT2 = xT1 + dirVec.x;
            float zT2 = zT1 + dirVec.z;

            //The final coordinates of the tangent lines
            pathData.startTangent = new Vector3(xT1, 0.1f, zT1);

            pathData.goalTangent = new Vector3(xT2, 0.1f, zT2);
        }



        //Inner tangent (RSL and LSR)
        public void RSLorLSR(
            Vector3 startCircle,
            Vector3 goalCircle,
            bool isBottom,
            OneReedsSheppPath pathData)
        {
            //Find the distance between the circles
            float D = (startCircle - goalCircle).magnitude;

            float R = turningRadius;

            //If the circles have the same radius we can use cosine and not the law of cosines 
            //to calculate the angle to the first tangent coordinate 
            float theta = Mathf.Acos((2f * R) / D);

            //If the circles is LSR, then the first tangent pos is on the other side of the center line
            if (isBottom)
            {
                theta *= -1f;
            }

            //Need to modify theta if the circles are not on the same height            
            theta += Mathf.Atan2(goalCircle.z - startCircle.z, goalCircle.x - startCircle.x);

            //The coordinates of the first tangent point
            float xT1 = startCircle.x + turningRadius * Mathf.Cos(theta);
            float zT1 = startCircle.z + turningRadius * Mathf.Sin(theta);

            //To get the second tangent coordinate we need the direction of the tangent
            //To get the direction we move up 2 circle radius and end up at this coordinate
            float xT1_tmp = startCircle.x + 2f * turningRadius * Mathf.Cos(theta);
            float zT1_tmp = startCircle.z + 2f * turningRadius * Mathf.Sin(theta);

            //The direction is between the new coordinate and the center of the target circle
            Vector3 dirVec = goalCircle - new Vector3(xT1_tmp, 0f, zT1_tmp);

            //The coordinates of the second tangent point is the 
            float xT2 = xT1 + dirVec.x;
            float zT2 = zT1 + dirVec.z;

            //The final coordinates of the tangent lines
            pathData.startTangent = new Vector3(xT1, 0.1f, zT1);

            pathData.goalTangent = new Vector3(xT2, 0.1f, zT2);
        }



        //CC|CC
        public void CC_turn_CC(
           Vector3 startCircle,
           Vector3 goalCircle,
           bool isBottom,
           OneReedsSheppPath pathData)
        {
            //The distance between the circles
            float D = (startCircle - goalCircle).magnitude;

            float r = turningRadius;

            float a = Mathf.Sqrt((2f * r * 2f * r) - ((r - (D * 0.5f)) * (r - (D * 0.5f))));

            //The angle we need to find the first circle center
            float theta = Mathf.Acos(a / (2f * r)) + (90f * Mathf.Deg2Rad);

            //Need to modify theta if the circles are not on the same height (z)
            float atan2 = Mathf.Atan2(goalCircle.z - startCircle.z, goalCircle.x - startCircle.x);

            if (isBottom)
            {
                theta = atan2 - theta;
            }
            else
            {
                theta = atan2 + theta;
            }

            //Center of the circle A
            float Ax = startCircle.x + 2f * r * Mathf.Cos(theta);
            float Az = startCircle.z + 2f * r * Mathf.Sin(theta);

            Vector3 circleAPos = new Vector3(Ax, 0f, Az);

            //The direction between the start circle and the goal circle
            //is the same as the direction between the outer circles
            Vector3 dirVec = (goalCircle - startCircle).normalized;

            //And the distance between the outer circles is 2r
            //So the position of the second circle is
            Vector3 circleBPos = circleAPos + (dirVec * 2f * r);

            //Now we can calculate the 3 tangent positions
            Vector3 dirVecA = (startCircle - circleAPos).normalized;
            Vector3 dirVecB = (goalCircle - circleBPos).normalized;

            Vector3 startTangent = circleAPos + (dirVecA * r);

            Vector3 middleTangent = circleAPos + (dirVec * r);

            Vector3 goalTangent = circleBPos + (dirVecB * r);

            //Save everything
            pathData.startTangent = startTangent;
            pathData.middleTangent = middleTangent;
            pathData.goalTangent = goalTangent;

            pathData.middleCircleCoordinate = circleAPos;
            pathData.middleCircleCoordinate2 = circleBPos;
        }



        //
        // Other calculations
        //

        //Calculate the length of an circle arc depending on which direction we are driving
        //isLeftCircle should be true if we are reversing in a right circle
        public float GetArcLength(
            Vector3 circleCenterPos,
            Vector3 startPos,
            Vector3 goalPos,
            OneReedsSheppSegment oneSegment)
        {
            //To get the arc length of each path we need to know which way we are turning
            bool isLeftCircle = IsTurningLeft(oneSegment.isTurningLeft, oneSegment.isReversing);

            Vector3 V1 = startPos - circleCenterPos;
            Vector3 V2 = goalPos - circleCenterPos;

            float theta = Mathf.Atan2(V2.z, V2.x) - Mathf.Atan2(V1.z, V1.x);

            if (theta < 0f && isLeftCircle)
            {
                theta += 2f * Mathf.PI;
            }
            else if (theta > 0 && !isLeftCircle)
            {
                theta -= 2f * Mathf.PI;
            }

            float arcLength = Mathf.Abs(theta * turningRadius);

            return arcLength;
        }



        //To get the arc length of each path we need to know which way we are turning
        bool IsTurningLeft(bool isTurningLeft, bool isReversing)
        {
            //Forward in a right circle is false
            //Reversing in a right circle is true
            //Reversing in a left circle is false
            //Forward in a left circle if true

            //L+
            if (isTurningLeft && !isReversing)
            {
                return isTurningLeft = true;
            }
            //L-
            if (isTurningLeft && isReversing)
            {
                return isTurningLeft = false;
            }
            //R+
            if (!isTurningLeft && !isReversing)
            {
                return isTurningLeft = false;
            }
            //R-
            if (!isTurningLeft && isReversing)
            {
                return isTurningLeft = true;
            }

            //To make C# happy but the above is enough
            return isTurningLeft;
        }



        //Loops through segments of a path and add new coordinates to the final path
        public void AddCoordinatesToPath(
            ref Vector3 currentPos,
            ref float theta,
            List<Node> finalPath,
            OneReedsSheppSegment segmentData)
        {
            //How far we are driving each update, the accuracy will improve if we lower the driveDistance
            //But not too low because then the path will be less accurate because of rounding errors
            float driveDistance = 0.02f;

            //How many segments has this line?
            int segments = Mathf.FloorToInt(segmentData.pathLength / driveDistance);


            //Always add the first position manually
            Node newNode = new Node();

            newNode.carPos = currentPos;
            newNode.heading = theta;

            if (segmentData.isReversing)
            {
                newNode.isReversing = true;
            }

            finalPath.Add(newNode);


            //Loop through all segments
            //i = 1 because we already added the first coordinate
            for (int i = 1; i < segments; i++)
            {
                //Can we improve the accuracy with Heuns method?
                //Probably not because speed is constant


                //Update the position
                if (segmentData.isReversing)
                {
                    currentPos.x -= driveDistance * Mathf.Sin(theta);
                    currentPos.z -= driveDistance * Mathf.Cos(theta);
                }
                else
                {
                    currentPos.x += driveDistance * Mathf.Sin(theta);
                    currentPos.z += driveDistance * Mathf.Cos(theta);
                }


                //Don't update the heading if we are driving straight
                if (segmentData.isTurning)
                {
                    //Which way are we turning?
                    float turnParameter = -1f;

                    if (!segmentData.isTurningLeft)
                    {
                        turnParameter = 1f;
                    }

                    if (segmentData.isReversing)
                    {
                        turnParameter *= -1f;
                    }

                    //Update the heading
                    theta += (driveDistance / turningRadius) * turnParameter;
                }


                //Add the new position and heading to the final path
                //Don't add all segments because 0.02 m per segment is too detailed
                //The real drive distance in the Hybrid A* tree is 1.05, so 52 = 1.05 / 0.02
                if (i % 52 == 0)
                {
                    newNode = new Node();

                    newNode.carPos = currentPos;
                    newNode.heading = theta;

                    if (segmentData.isReversing)
                    {
                        newNode.isReversing = true;
                    }

                    finalPath.Add(newNode);
                }
            }
        }
    }
}
