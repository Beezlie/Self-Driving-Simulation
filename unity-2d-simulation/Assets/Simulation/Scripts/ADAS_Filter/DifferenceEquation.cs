using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//references https://bitbucket.org/dshin-uwaterloo/adas-car-on-treadmill-v2/src/master/src/adas_filters/include/adas_filters/difference_equation.hpp
public class DifferenceEquation {

    private float[] inCoeffs;
    private float[] outCoeffs;
    private float[] inStates;
    private float[] outStates;
    private ArrayList ins;
    private ArrayList outs;

    // Ctor with set initial conditions
    public DifferenceEquation(float[] inCoeffs, float[] outCoeffs, float[] inInitials, float[] outInitials)
    {
        this.inCoeffs = new float[inCoeffs.Length];
        Array.Copy(inCoeffs, this.inCoeffs, inCoeffs.Length);
        this.outCoeffs = new float[outCoeffs.Length];
        Array.Copy(outCoeffs, this.outCoeffs, outCoeffs.Length);
        ins = new ArrayList(inInitials);
        outs = new ArrayList(outInitials);

        if (inCoeffs.Length <= 0) {
            throw new System.ArgumentException("Number in coefficients must be larger than zero.");
        }
    }

    // Ctor with initial conditions of 0
    public DifferenceEquation(float[] inCoeffs, float[] outCoeffs)
    {
        this.inCoeffs = new float[inCoeffs.Length];
        Array.Copy(inCoeffs, this.inCoeffs, inCoeffs.Length);
        this.outCoeffs = new float[outCoeffs.Length];
        Array.Copy(outCoeffs, this.outCoeffs, outCoeffs.Length);
        ins = new ArrayList();
        outs = new ArrayList();

        if (inCoeffs.Length <= 0) {
            throw new System.ArgumentException("Number in coefficients must be larger than zero.");
        }
    }

    public void AssignState(float[] inStates, float[] outStates)
    {
        this.inStates = new float[inStates.Length];
        Array.Copy(inStates, this.inStates, inStates.Length);
        this.outStates = new float[outStates.Length];
        Array.Copy(outStates, this.outStates, outStates.Length);
    }

    public void AssignState(DifferenceEquation other)
    {
        AssignState(other.inStates, other.outStates);
    }

public void AssignCoeffs(float[] inCoeffs, float[] outCoeffs)
    {
        this.inCoeffs = new float[inCoeffs.Length];
        Array.Copy(inCoeffs, this.inCoeffs, inCoeffs.Length);
        this.outCoeffs = new float[outCoeffs.Length];
        Array.Copy(outCoeffs, this.outCoeffs, outCoeffs.Length);
    }

    public float Output(float input)
    {
        ins.Insert(0, input);

        // Calculate inner products
        float sumIns = 0;
        for (int i = 0; i < inCoeffs.Length && i < ins.Count; i++) {
            sumIns += inCoeffs[i] * (float)ins[i];
        }
        float sumOuts = 0;
        for (int i = 0; i < outCoeffs.Length && i < outs.Count; i++)
        {
            sumOuts += outCoeffs[i] * (float)outs[i];
        }

        float newOut = sumIns - sumOuts;
        outs.Insert(0, newOut);
        return newOut;
    }
}
