using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AsymmetricFirstOrderSystem {

    private float k;
    private float increaseTau;
    private float decreaseTau;
    private float samplingFreqHz;
    private float initialOutput;
    private float prev;
    private bool beenIncreasing;
    private FirstOrderSystem increaseSystem;
    private FirstOrderSystem decreaseSystem;

    public AsymmetricFirstOrderSystem(float k, float increaseTau, float decreaseTau, float samplingFreqHz, float initialOutput)
    {
        Debug.Log("Creating sys");
        prev = 0f;
        beenIncreasing = true;
        increaseSystem = new FirstOrderSystem(k, increaseTau, samplingFreqHz, initialOutput);
        decreaseSystem = new FirstOrderSystem(k, decreaseTau, samplingFreqHz, initialOutput);
    }

    public float Output(float input)
    {
        FirstOrderSystem selected = Select(input);
        return selected.Output(input);
    }

    public void Reset()
    {
        increaseSystem.Reset();
        decreaseSystem.Reset();
    }

    public void Reset(float newInitialOutput)
    {
        increaseSystem.Reset(newInitialOutput);
        decreaseSystem.Reset(newInitialOutput);
    }

    public FirstOrderSystem Select(float curr)
    {
        if (prev < curr && !beenIncreasing) {
            // Now increasing. Inherit the state from the decreasing system.
            beenIncreasing = true;
            increaseSystem.AssignState(decreaseSystem);
        }
        else if (prev > curr && beenIncreasing) {
            // Now decreasing. Inherit the state from the increasing system.
            beenIncreasing = false;
            decreaseSystem.AssignState(increaseSystem);
        }

        if (beenIncreasing) {
            return increaseSystem;
        }
        return decreaseSystem;
    }
}
