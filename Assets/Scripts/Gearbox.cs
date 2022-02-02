using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Gearbox : MonoBehaviour
{
    public VehicleController vehicleController;
    Engine engine;

    public float[] gearRatios;
    public float differentialRatio = 3.73f;

    void Start() {
        engine = vehicleController.engine;

    }

    public float TorqueOnAxis() {
        float torque = engine.GetTorque() * differentialRatio * gearRatios[vehicleController.currentGear];
        return torque;
    }

    public float GetRPM() {
        float rpm = 0;
        foreach(Wheel w in vehicleController.wheels) {
            if(w.rl) {
                rpm = w.GetAngularVelocity() * 60 / (2 * Mathf.PI);
                rpm = rpm * differentialRatio * gearRatios[vehicleController.currentGear];
            }
        }
        if(rpm < 1000) rpm = 1000;
        return rpm;
    }
}
