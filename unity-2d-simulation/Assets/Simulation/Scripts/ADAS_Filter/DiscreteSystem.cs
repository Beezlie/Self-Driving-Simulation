using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//references https://bitbucket.org/dshin-uwaterloo/adas-car-on-treadmill-v2/src/master/src/adas_filters/include/adas_filters/discrete_system.hpp
public class DiscreteSystem {

    private float[] inCoeffs;
    private float[] outCoeffs;
    private float initialOutput;
    private DifferenceEquation eqn;
    private int numInAndPrevs = 2;
    private int numOutAndPrevs = 1;

    public DiscreteSystem(float[] inCoeffs, float[] outCoeffs, float initialOutput)
    {
        Debug.Log((string.Join(",", inCoeffs)));
        Debug.Log((string.Join(",", outCoeffs)));

        this.inCoeffs = new float[inCoeffs.Length];
        Array.Copy(inCoeffs, this.inCoeffs, inCoeffs.Length);
        Debug.Log("copied inCoeffs");
        this.outCoeffs = new float[outCoeffs.Length];
        Array.Copy(outCoeffs, this.outCoeffs, outCoeffs.Length);
        Debug.Log("copied outCoeffs");

        this.initialOutput = initialOutput;
        eqn = new DifferenceEquation(inCoeffs, outCoeffs);

        Reset();
    }

    public float Output(float input)
    {
        return eqn.Output(input);
    }

    public void Reset(float newInitialOutput)
    {
        initialOutput = newInitialOutput;
        Reset();
    }

    public void Reset()
    {
        float[] inStates = new float[numInAndPrevs];
        float[] outStates = new float[numOutAndPrevs];

        // No input is assumed.
        for (int i = 0; i < inStates.Length; i++) {
            inStates[i] = 0f;
        }
        // System is at rest, at the initial value.
        for (int i = 0; i < outStates.Length; i++) {
            outStates[i] = initialOutput;
        }

        eqn.AssignState(inStates, outStates);
    }

    public void AssignState(DiscreteSystem other)
    {
        eqn.AssignState(other.eqn);
    }
}
