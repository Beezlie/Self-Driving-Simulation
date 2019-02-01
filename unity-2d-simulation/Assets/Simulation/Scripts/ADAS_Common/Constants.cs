﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// references https://bitbucket.org/dshin-uwaterloo/adas-car-on-treadmill-v2/src/master/config/vrep_sim/params.yaml
public static class Constants {

    public static float steeringCommandMin = -Mathf.PI / 6f;
    public static float steeringCommandNeutral = 0f;
    public static float steeringCommandMax = steeringCommandMin;
    public static float throttleCommandMin = 0f;
    public static float throttleCommandMax = 100f;

    public static float neutralHeading = 270f;

    //TODO - find out how to make these dynamically configurable

    // Car controller constants
    public static float rearAxleFromTag = -0.05475f;
    public static ArrayList boundingBox = new ArrayList { 0.0f, 0.0f, 0.14155f, 0.1002f };
    public static float goalDistanceLimit = 0.36f;

    public static float pVel = 2.5f;
    public static float iVel = 1f;
    public static float dVel = 0f;
    public static float pVelUrgent = 2.5f;
    public static float iVelUrgent = 1f;
    public static float dVelUrgent = 0f;
    public static float velMin = -3f;
    public static float velMax = 3f;
    public static float velMinUrgent = -4f;
    public static float velMaxUrgent = 4f;
    public static float kFeedForward = 16.6f;

    public static float pPos = 1f;
    public static float iPos = 0.025f;
    public static float dPos = 0.75f;
    public static float pPosUrgent = 1f;
    public static float iPosUrgent = 0.025f;
    public static float dPosUrgent = 0.75f;
    public static float discontinuityThreshold = 0.05f;
    public static float iThreshold = 0.1f;

    public static float kpHeading = 0.5f;
    public static float kdHeading = 0.01f;
    public static float kCrosstrack = 1f;
    public static float kCrosstrackUrgent = 1.2f;
    public static float velDamping = 1f;
    public static float axleDistance = 0.099f;

    public static float carSimVelK = 0.06f;
    public static float carSimVelIncreaseTau = 1f;
    public static float carSimVelDecreaseTau = 2f;
    public static float carSimSteeringK = 0.5235f;
    public static float carSimSteeringTau = 0.1f;

    // Track controller constants
    public static float trackSlope = 45.92f;
    public static float trackIntercept = 0f;
    public static float pTrack = 10f;
    public static float iTrack = 5f;
    public static float dTrack = 0f;

    public static float trackSimK = 0.0215873f;
    public static float trackSimIncreaseTau = 0.2425f;
    public static float trackSimDecreaseTau = 0.45f;
    public static ArrayList lanes = new ArrayList { 0.24f, 0.48f, 0.72f, 0.96f, 1.2f };
    public static float laneInterval = 0.2f;
    public static float trackLength = 2.26f;
    public static float targetHz = 20f;

}
 