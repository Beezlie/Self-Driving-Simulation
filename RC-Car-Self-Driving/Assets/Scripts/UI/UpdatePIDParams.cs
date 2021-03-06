﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class UpdatePIDParams : MonoBehaviour {

    private InputField pVel;
    private InputField iVel;
    private InputField dVel;
    private InputField pPos;
    private InputField iPos;
    private InputField dPos;
    private InputField kFF;
    private InputField kCT;
    private InputField kpHeading;
    private InputField kdHeading;
    private InputField velDamping;

    private GameObject tuningCar;
    private TuningCarInterface tuningCarInterface;

    void Awake () {
        //Get reference to tuning car
        tuningCar = GameObject.Find("TuningCar");
        if (tuningCar == null)
        {
            Debug.Log("The TuningCar object was not found.");
        }
        tuningCarInterface = tuningCar.GetComponent<TuningCarInterface>();

        //pVel 
        pVel = GameObject.Find("pVelField").GetComponent<InputField>();
        pVel.text = Constants.pVel.ToString();
        pVel.onValueChanged.AddListener(delegate { pVelValueChangeCheck(); });

        //iVel 
        iVel = GameObject.Find("iVelField").GetComponent<InputField>();
        iVel.text = Constants.iVel.ToString();
        iVel.onValueChanged.AddListener(delegate { iVelValueChangeCheck(); });

        //dVel 
        dVel = GameObject.Find("dVelField").GetComponent<InputField>();
        dVel.text = Constants.dVel.ToString();
        dVel.onValueChanged.AddListener(delegate { dVelValueChangeCheck(); });

        //pPos
        pPos = GameObject.Find("pPosField").GetComponent<InputField>();
        pPos.text = Constants.pPos.ToString();
        pPos.onValueChanged.AddListener(delegate { pPosValueChangeCheck(); });

        //iPos
        iPos = GameObject.Find("iPosField").GetComponent<InputField>();
        iPos.text = Constants.iPos.ToString();
        iPos.onValueChanged.AddListener(delegate { iPosValueChangeCheck(); });

        //dPos
        dPos = GameObject.Find("dPosField").GetComponent<InputField>();
        dPos.text = Constants.dPos.ToString();
        dPos.onValueChanged.AddListener(delegate { dPosValueChangeCheck(); });

        //kFF
        kFF = GameObject.Find("kFFField").GetComponent<InputField>();
        kFF.text = Constants.kFeedForward.ToString();
        kFF.onValueChanged.AddListener(delegate { kFFValueChangeCheck(); });

        //kCT
        kCT = GameObject.Find("kCTField").GetComponent<InputField>();
        kCT.text = Constants.kCrosstrack.ToString();
        kCT.onValueChanged.AddListener(delegate { kCTValueChangeCheck(); });

        //kpHeading
        kpHeading = GameObject.Find("kpHeadingField").GetComponent<InputField>();
        kpHeading.text = Constants.kpHeading.ToString();
        kpHeading.onValueChanged.AddListener(delegate { kpHeadingValueChangeCheck(); });

        //kdHeading
        kdHeading = GameObject.Find("kdHeadingField").GetComponent<InputField>();
        kdHeading.text = Constants.kdHeading.ToString();
        kdHeading.onValueChanged.AddListener(delegate { kdHeadingValueChangeCheck(); });

        //velDamping
        velDamping = GameObject.Find("velDampingField").GetComponent<InputField>();
        velDamping.text = Constants.velDamping.ToString();
        velDamping.onValueChanged.AddListener(delegate { velDampingValueChangeCheck(); });
    }

    private void pVelValueChangeCheck()
    {
        Constants.pVel = float.Parse(pVel.text);
        tuningCarInterface.ResetParams();
    }

    private void iVelValueChangeCheck()
    {
        Constants.iVel = float.Parse(iVel.text);
        tuningCarInterface.ResetParams();
    }

    private void dVelValueChangeCheck()
    {
        Constants.dVel = float.Parse(dVel.text);
        tuningCarInterface.ResetParams();
    }

    private void pPosValueChangeCheck()
    {
        Constants.pPos = float.Parse(pPos.text);
        tuningCarInterface.ResetParams();
    }

    private void iPosValueChangeCheck()
    {
        Constants.iPos = float.Parse(iPos.text);
        tuningCarInterface.ResetParams();
    }

    private void dPosValueChangeCheck()
    {
        Constants.dPos = float.Parse(dPos.text);
        tuningCarInterface.ResetParams();
    }

    private void kFFValueChangeCheck()
    {
        Constants.kFeedForward = float.Parse(kFF.text);
        tuningCarInterface.ResetParams();
    }

    private void kCTValueChangeCheck()
    {
        Constants.kCrosstrack = float.Parse(kCT.text);
        tuningCarInterface.ResetParams();
    }

    private void kdHeadingValueChangeCheck()
    {
        Constants.kdHeading = float.Parse(kdHeading.text);
        tuningCarInterface.ResetParams();
    }

    private void kpHeadingValueChangeCheck()
    {
        Constants.kpHeading = float.Parse(kpHeading.text);
        tuningCarInterface.ResetParams();
    }

    private void velDampingValueChangeCheck()
    {
        Constants.velDamping = float.Parse(velDamping.text);
        tuningCarInterface.ResetParams();
    }
}
