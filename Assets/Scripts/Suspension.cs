using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Suspension : MonoBehaviour
{
    VehicleController vehicleController;
    CarBody carBody;

    public float springLength = 0.2f;
    public float springConstant = 80000;
    public float dampingForce;
    float springPos;

    //public float springPos;

    void Start() {
        vehicleController = GetComponent<VehicleController>();
        carBody = vehicleController.carBody;
    }

    public float GetSpringPosFront() {
        float axleWeight = 0;

        foreach(Wheel w in vehicleController.wheels) {
            if(w.fl || w.fr) {
                axleWeight += w.GetWeightOnWheel();
            }
        }
        float springPos = axleWeight / springConstant;
        if(springPos < 0) springPos = 0;
        if(springPos > springLength) springPos = springLength;

        return springPos;
    }

    public float GetSpringPosRear() {
        float axleWeight = 0;

        foreach(Wheel w in vehicleController.wheels) {
            if(w.rl || w.rr) {
                axleWeight += w.GetWeightOnWheel();
            }
        }
        float springPos = axleWeight / springConstant;
        if(springPos < 0) springPos = 0;
        if(springPos > springLength) springPos = springLength;

        return springPos;
    }

    public float GetSpringPosLeft() {
        float sideWeight = 0;

        foreach(Wheel w in vehicleController.wheels) {
            if(w.fl || w.rl) {
                sideWeight += w.GetWeightOnWheel();
            }
        }
        float springPos = sideWeight / springConstant;
        if(springPos < 0) springPos = 0;
        if(springPos > springLength) springPos = springLength;

        return springPos;
    }

    public float GetSpringPosRight() {
        float sideWeight = 0;

        foreach(Wheel w in vehicleController.wheels) {
            if(w.fr || w.rr) {
                sideWeight += w.GetWeightOnWheel();
            }
        }
        float springPos = sideWeight / springConstant;
        if(springPos < 0) springPos = 0;
        if(springPos > springLength) springPos = springLength;
        
        return springPos;
    }

    float GetDampingRatio() {
        float dampingCoefficient = dampingForce / (0 - 0); // 0 - 0 = Carbody vertical speed - wheel vertical speed
        float dampingRatio = dampingCoefficient / (2 * Mathf.Sqrt(springConstant * carBody.mass));
        return dampingRatio;
    }

    float GetSpringPos() {
        //float magnitudeOfResponse = 0;
        //float naturalFreq = Mathf.Sqrt(1f - );
        //float phaseAngle = Mathf.Atan();


        return 0;
    }
}
