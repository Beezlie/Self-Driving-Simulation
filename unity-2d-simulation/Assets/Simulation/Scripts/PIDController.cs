using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PIDController {

    public float p;
    public float i;
    public float d;
    private float saturationMin;
    private float saturationMax;
    private bool prevValid = false;
    private bool goalValid = false;
    private bool accumulate = true;
    private float goal;
    private float accum;
    private float prevError;

    public PIDController(float p, float d, float i, float saturationMin, float saturationMax)
    {
        this.p = p;
        this.i = i;
        this.d = d;
        this.saturationMin = saturationMin;
        this.saturationMax = saturationMax;
        prevValid = false;
        goalValid = false;
        accumulate = true;
        accum = 0f;

        checkSaturation();
    }

    public void setGoal(float r)
    {
        goalValid = true;
        goal = r;
    }

    public void setSaturation(float min, float max)
    {
        saturationMin = min;
        saturationMax = max;

        // Inherit the accumulation status, but trim the accum value
        accum = Mathf.Clamp(accum, saturationMin, saturationMax);

        checkSaturation();
    }

    private void checkSaturation()
    {
        if (saturationMin > saturationMax) {
            throw new System.ArgumentException("Saturation min value must be smaller than the max value.");
        }
    }

    public float commandStep(float feedforward, float y, float dt, bool doAccumulate)
    {
        if (!goalValid) {
            return 0f;
        }

        float e = goal - y;

        float pTerm = p * e;
        float totalTerm = pTerm + feedforward;

        if (!prevValid) {
            prevError = e;
            prevValid = true;
            return Mathf.Clamp(totalTerm, saturationMin, saturationMax);
        }

        // Accumulate if told by user AND windup prevention not in place
        if (doAccumulate && accumulate) {
            // Trapezoidal integration
            accum += (prevError + e) / 2 * dt;
        }

        float iTerm = i * accum;
        float dTerm = d * (e - prevError) / dt;

        totalTerm += iTerm + dTerm;

        // Update prev
        prevError = e;

        // Update windup prevention
        bool saturating = totalTerm < saturationMin || totalTerm > saturationMax;
        bool errorAccumSignMatch = accum * prevError > 0;
        accumulate = !(saturating && errorAccumSignMatch);

        return Mathf.Clamp(totalTerm, saturationMin, saturationMax);
    }

    public void reset()
    {
        accum = 0f;
        prevValid = false;
        accumulate = true;
    }
}
