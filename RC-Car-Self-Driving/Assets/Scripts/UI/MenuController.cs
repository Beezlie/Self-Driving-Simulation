using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class MenuController : MonoBehaviour {

    private GameObject pidTuningMenu;
    private GameObject goalSliders;
    private bool pidIsShowing = false;
    private bool goalSliderIsShowing = false;

    private void Awake()
    {
        pidTuningMenu = GameObject.Find("TuneUI");
        if (pidTuningMenu == null)
        {
            Debug.Log("The TuneUI object was not found.");
        }
        goalSliders = GameObject.Find("GoalUI");
        if (goalSliders == null)
        {
            Debug.Log("The GoalUI object was not found.");
        }
    }

    void Start () {
        pidTuningMenu.SetActive(pidIsShowing);
        goalSliders.SetActive(pidIsShowing);
    }

    void Update()
    {
        if (Input.GetKey("t"))
        {
            Debug.Log("P KEY PRESSED");
            pidIsShowing = !pidIsShowing;
            pidTuningMenu.SetActive(pidIsShowing);
        } else if(Input.GetKey("g"))
        {
            Debug.Log("G KEY PRESSED");
            goalSliderIsShowing = !goalSliderIsShowing;
            goalSliders.SetActive(goalSliderIsShowing);
        }
    }
}
