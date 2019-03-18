using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class UpdateGoalParams : MonoBehaviour {

    private Slider xPos;
    private Slider zPos;
    private GameObject road;

    private void Awake()
    {
        //Get reference to track
        road = GameObject.Find("Road Piece");
        if (road == null)
        {
            Debug.Log("The Road Piece object was not found.");
        }

        //lateral goal position 
        xPos = GameObject.Find("LateralSlider").GetComponent<Slider>();
        xPos.onValueChanged.AddListener(delegate { xPosValueChangeCheck(); });

        //longitudinal goal position 
        zPos = GameObject.Find("LongitudinalSlider").GetComponent<Slider>();
        zPos.onValueChanged.AddListener(delegate { zPosValueChangeCheck(); });
    }

    private void Start()
    {
        //set min/max positions for x and z
        float offset = 1f;
        xPos.minValue = 0 + offset;
        xPos.maxValue = road.gameObject.GetComponent<MeshRenderer>().bounds.size.x - offset;
        zPos.minValue = 0 + offset;
        zPos.maxValue = road.gameObject.GetComponent<MeshRenderer>().bounds.size.z - offset;
    }

    private void xPosValueChangeCheck()
    {
        Constants.goalCoordinates.x = xPos.value;
    }

    private void zPosValueChangeCheck()
    {
        Constants.goalCoordinates.z = zPos.value;
    }
}
