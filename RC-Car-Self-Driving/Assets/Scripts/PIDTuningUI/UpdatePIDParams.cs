using System.Collections;
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
    private InputField velMin;
    private InputField velMax;
    private InputField kFF;
    private InputField kCT;
    private InputField discThresh;
    private InputField iThresh;
    private InputField kpHeading;
    private InputField kdHeading;
    private InputField velDamping;
    private InputField targetHz;
    private InputField carVelK;
    private InputField carVelITau;
    private InputField carVelDTau;
    private InputField carSteerK;
    private InputField carSteerTau;
    private InputField goalX;
    private InputField goalY;
    private InputField goalZ;

    void Awake () {
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

        //velMin
        velMin = GameObject.Find("velMinField").GetComponent<InputField>();
        velMin.text = Constants.velMin.ToString();
        velMin.onValueChanged.AddListener(delegate { velMinValueChangeCheck(); });

        //velMax
        velMax = GameObject.Find("velMaxField").GetComponent<InputField>();
        velMax.text = Constants.velMax.ToString();
        velMax.onValueChanged.AddListener(delegate { velMaxValueChangeCheck(); });

        //kFF
        kFF = GameObject.Find("kFFField").GetComponent<InputField>();
        kFF.text = Constants.kFeedForward.ToString();
        kFF.onValueChanged.AddListener(delegate { kFFValueChangeCheck(); });

        //kCT
        kCT = GameObject.Find("kCTField").GetComponent<InputField>();
        kCT.text = Constants.kCrosstrack.ToString();
        kCT.onValueChanged.AddListener(delegate { kCTValueChangeCheck(); });

        //discThresh
        discThresh = GameObject.Find("discThreshField").GetComponent<InputField>();
        discThresh.text = Constants.discontinuityThreshold.ToString();
        discThresh.onValueChanged.AddListener(delegate { discThreshValueChangeCheck(); });

        //iThresh
        iThresh = GameObject.Find("iThreshField").GetComponent<InputField>();
        iThresh.text = Constants.iThreshold.ToString();
        iThresh.onValueChanged.AddListener(delegate { iThreshValueChangeCheck(); });

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

        //targetHz
        targetHz = GameObject.Find("targetHzField").GetComponent<InputField>();
        targetHz.text = Constants.targetHz.ToString();
        targetHz.onValueChanged.AddListener(delegate { targetHzValueChangeCheck(); });

        //carVelK
        carVelK = GameObject.Find("carVelKField").GetComponent<InputField>();
        carVelK.text = Constants.carSimVelK.ToString();
        carVelK.onValueChanged.AddListener(delegate { carVelKValueChangeCheck(); });

        //carVelITau
        carVelITau = GameObject.Find("carVelITauField").GetComponent<InputField>();
        carVelITau.text = Constants.carSimVelIncreaseTau.ToString();
        carVelITau.onValueChanged.AddListener(delegate { carVelITauValueChangeCheck(); });

        //carVelDTau
        carVelDTau = GameObject.Find("carVelDTauField").GetComponent<InputField>();
        carVelDTau.text = Constants.carSimVelDecreaseTau.ToString();
        carVelDTau.onValueChanged.AddListener(delegate { carVelDTauValueChangeCheck(); });

        //carSteerK
        carSteerK = GameObject.Find("carSteerKField").GetComponent<InputField>();
        carSteerK.text = Constants.carSimSteeringK.ToString();
        carSteerK.onValueChanged.AddListener(delegate { carSteerKValueChangeCheck(); });

        //carSteerTau
        carSteerTau = GameObject.Find("carSteerTauField").GetComponent<InputField>();
        carSteerTau.text = Constants.carSimSteeringTau.ToString();
        carSteerTau.onValueChanged.AddListener(delegate { carSteerTauValueChangeCheck(); });

        //goalX
        goalX = GameObject.Find("goalXField").GetComponent<InputField>();
        goalX.text = Constants.goalCoordinates.x.ToString();
        goalX.onValueChanged.AddListener(delegate { goalXValueChangeCheck(); });

        //goalY
        goalY = GameObject.Find("goalYField").GetComponent<InputField>();
        goalY.text = Constants.goalCoordinates.y.ToString();
        goalY.onValueChanged.AddListener(delegate { goalYValueChangeCheck(); });

        //goalZ
        goalZ = GameObject.Find("goalZField").GetComponent<InputField>();
        goalZ.text = Constants.goalCoordinates.z.ToString();
        goalZ.onValueChanged.AddListener(delegate { goalZValueChangeCheck(); });
    }

    private void pVelValueChangeCheck()
    {
        Constants.pVel = float.Parse(pVel.text);
    }

    private void iVelValueChangeCheck()
    {
        Constants.iVel = float.Parse(iVel.text);
    }

    private void dVelValueChangeCheck()
    {
        Constants.dVel = float.Parse(dVel.text);
    }

    private void pPosValueChangeCheck()
    {
        Constants.pPos = float.Parse(pPos.text);
    }

    private void iPosValueChangeCheck()
    {
        Constants.iPos = float.Parse(iPos.text);
    }

    private void dPosValueChangeCheck()
    {
        Constants.dPos = float.Parse(dPos.text);
    }

    private void velMinValueChangeCheck()
    {
        Constants.velMin = float.Parse(velMin.text);
    }

    private void velMaxValueChangeCheck()
    {
        Constants.velMax = float.Parse(velMax.text);
    }

    private void kFFValueChangeCheck()
    {
        Constants.kFeedForward = float.Parse(kFF.text);
    }

    private void kCTValueChangeCheck()
    {
        Constants.kCrosstrack = float.Parse(kCT.text);
    }

    private void discThreshValueChangeCheck()
    {
        Constants.discontinuityThreshold = float.Parse(discThresh.text);
    }

    private void iThreshValueChangeCheck()
    {
        Constants.iThreshold = float.Parse(iThresh.text);
    }

    private void kdHeadingValueChangeCheck()
    {
        Constants.kdHeading = float.Parse(kdHeading.text);
    }

    private void kpHeadingValueChangeCheck()
    {
        Constants.kpHeading = float.Parse(kpHeading.text);
    }

    private void velDampingValueChangeCheck()
    {
        Constants.velDamping = float.Parse(velDamping.text);
    }

    private void targetHzValueChangeCheck()
    {
        Constants.targetHz = float.Parse(targetHz.text);
    }

    private void carVelKValueChangeCheck()
    {
        Constants.carSimVelK = float.Parse(carVelK.text);
    }

    private void carVelITauValueChangeCheck()
    {
        Constants.carSimVelIncreaseTau = float.Parse(carVelITau.text);
    }

    private void carVelDTauValueChangeCheck()
    {
        Constants.carSimVelDecreaseTau = float.Parse(carVelDTau.text);
    }

    private void carSteerKValueChangeCheck()
    {
        Constants.carSimSteeringK = float.Parse(carSteerK.text);
    }

    private void carSteerTauValueChangeCheck()
    {
        Constants.carSimSteeringTau = float.Parse(carSteerTau.text);
    }

    private void goalXValueChangeCheck()
    {
        Constants.goalCoordinates.x = float.Parse(goalX.text);
    }

    private void goalYValueChangeCheck()
    {
        Constants.goalCoordinates.y = float.Parse(goalY.text);
    }

    private void goalZValueChangeCheck()
    {
        Constants.goalCoordinates.z = float.Parse(goalZ.text);
    }
}
