using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FirstOrderSystem {

    private float k;
    private float tau;
    private float samplingFreqHz;
    private float initialOutput;
    private DiscreteSystem discreteSystem;

    public FirstOrderSystem(float k, float tau, float samplingFreqHz, float initialOutput)
    {
        this.k = k;
        this.tau = tau;
        this.samplingFreqHz = samplingFreqHz;
        this.initialOutput = initialOutput;

        float T = 1 / samplingFreqHz;
        float denom = 2 * tau + T;
        float inCoeff = k * T / denom;

        float[] inCoeffs = new float[] { inCoeff, inCoeff };
        float[] outCoeffs = new float[] { (T - 2 * tau) / denom };

        discreteSystem = new DiscreteSystem(inCoeffs, outCoeffs, initialOutput);
    }

    public void Reset()
    {
        discreteSystem.Reset();
    }

    public void Reset(float newInitialOutput)
    {
        discreteSystem.Reset(newInitialOutput);
    }

    public void AssignState(FirstOrderSystem other)
    {
        discreteSystem.AssignState(other.discreteSystem);
    }

    public float Output(float input)
    {
        return discreteSystem.Output(input);
    }
}
