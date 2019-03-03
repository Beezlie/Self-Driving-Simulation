using UnityEngine;

public class CarState
{

    public float v;
    public float z;
    public float x;
    public float psi;

    public float dz;
    public float dx;
    public float dpsi;
    public float dv;

    private float acceleration = 3;
    private float lr;
    private float lf;

    public CarState(float v, float z, float x, float psi, float lr, float lf)
    {
        this.v = v;
        this.z = z;
        this.x = x;
        this.psi = psi;
        this.lr = lr;
        this.lf = lf;

        Debug.Log(string.Format("setting v as {0}", v));
        Debug.Log(string.Format("setting z as {0}", z));
        Debug.Log(string.Format("setting x as {0}", x));
        Debug.Log(string.Format("setting psi as {0}", psi));
    }

    public void UpdateState(float throttle, float steer, float dt, float trackVel)
    {
        float beta = Mathf.Atan((lr / (lf + lr)) * Mathf.Tan(steer));   //steer is in radians

        // calculate incremental changes
        float vnew = v * acceleration;
        dz = vnew * Mathf.Cos(psi + beta) - trackVel;
        dx = vnew * Mathf.Sin(psi + beta);
        dpsi = (vnew / lr) * Mathf.Sin(beta);
        dv = 0;

        // update state values
        z = z + dz * dt;
        x = x + dx * dt;
        psi = psi + dpsi * dt;

        v = throttle;       //might need to multiply by some constant here

        Debug.Log(string.Format("lr = lf: {0}", lr));
        Debug.Log(string.Format("beta: {0}", beta));
        Debug.Log(string.Format("dx: {0}", dz));
        Debug.Log(string.Format("dy: {0}", dx));
        Debug.Log(string.Format("dpsi: {0}", dpsi));
        Debug.Log(string.Format("dv: {0}", dv));
        Debug.Log(string.Format("z: {0}", z));
        Debug.Log(string.Format("x: {0}", x));
        Debug.Log(string.Format("psi: {0}", psi));
        Debug.Log(string.Format("v: {0}", v));
    }
}
