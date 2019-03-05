using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class TuneParamsMenu : MonoBehaviour {

    public GameObject menu;
    private bool isShowing = false;

    void Start () {
        menu.SetActive(isShowing);
	}
	
    void Update()
    {
        if (Input.GetKey("escape"))
        {
            Debug.Log("KEY PRESSED");
            isShowing = !isShowing;
            menu.SetActive(isShowing);
        }
    }
}
