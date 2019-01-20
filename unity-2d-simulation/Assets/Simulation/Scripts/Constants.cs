using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class Constants {

    public static float steeringCommandMin = -Mathf.PI / 6f;
    public static float steeringCommandNeutral = 0f;
    public static float steeringCommandMax = steeringCommandMin;
    public static float throttleCommandMin = 0f;
    public static float throttleCommandMax = 100f;
}
