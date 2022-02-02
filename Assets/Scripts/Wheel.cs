using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Wheel : MonoBehaviour
{
    public VehicleController vehicleController;
    CarBody carBody;
    Gearbox gearbox;
    public float mass;
    public float radius;
    public bool fl, fr, rl, rr;
    public AnimationCurve coefficientFrictionByTemperature;
    public AnimationCurve coefficientFrictionBySlipAngle;
    public float coefficientOfFrictionForward;
    public float coefficientOfFrictionForwardOnSlip;
    public float coefficientOfRollingFrictiontion = 0.015f;
    public float corneringStiffness;
    public float lateralVelocity;
    public float slipAngle;
    float weightOnWheel = 0;
    
    void Start() {
        carBody = vehicleController.carBody;
        gearbox = vehicleController.gearbox;
    }

    public float GetRollingResistance() {
        float rollingResistanceForce = Mathf.Sign(vehicleController.velocity.x) * (GetWeightOnWheel() * coefficientOfRollingFrictiontion) 
            / Mathf.Sqrt((radius * radius) - (coefficientOfRollingFrictiontion * coefficientOfRollingFrictiontion));

        return rollingResistanceForce;
    }

    public float GetWeightOnWheel() {
        bool isOutSideWheel = false;
        if(vehicleController.force.y > 0) {
            if(rl || fl) {
                isOutSideWheel = true;
            }
        }
        
        if(vehicleController.force.y < 0) {
            if(rr || fr) {
                isOutSideWheel = true;
            }
        }

        weightOnWheel = 0;
        if(rl || rr) {
            // Static wheel load
            weightOnWheel = carBody.mass * Environment.gravity * (carBody.fromCenterToFrontAxle / carBody.wheelBase);

            // Affection of Forward acceleration
            weightOnWheel = weightOnWheel + ((carBody.centerOfGravityHeight / carBody.wheelBase) * (vehicleController.force.x / 2)); 
        }
        else {
            // Static wheel load
            weightOnWheel = carBody.mass * Environment.gravity * (carBody.fromCenterToRearAxle / carBody.wheelBase);

            // Affection of Forward acceleration
            weightOnWheel = weightOnWheel - ((carBody.centerOfGravityHeight / carBody.wheelBase) * (vehicleController.force.x / 2)); // / 4 pois?
        }

        // Affection of cornering ; / 2 assuming center of gravity is in center; 

        if(isOutSideWheel) {
            weightOnWheel = (weightOnWheel / 2) + ((Mathf.Abs(vehicleController.force.y) / 2) * (carBody.centerOfGravityHeight / carBody.fromLeftWheelToRight));
        }
        else {
            weightOnWheel = (weightOnWheel / 2) - ((Mathf.Abs(vehicleController.force.y) / 2) * (carBody.centerOfGravityHeight / carBody.fromLeftWheelToRight)); 
        }
        



        // Missing: Torsionally Compliant Chassis Weight Transfer, affection of cornering to load on front and rear.

        return weightOnWheel;
    }

    public float GetTorqueOnWheel() {
        float torque = 0;
        if(rl || rr) {
            torque = gearbox.TorqueOnAxis() / 2;
        }
        return torque;
    }

    public float GetWheelForwardTractionForce() {
        float force = 0;
        if(rl || rr) {
            force = GetTorqueOnWheel() / radius;
        }
        return force;
    }

    public float GetAngularVelocity() {
        float angularVelocity = vehicleController.velocity.x / radius;

        if(rr || rl) {
            angularVelocity += GetRotationalAcceleration() * Time.deltaTime;
        }

        return angularVelocity;
    }

    public float GetRotationalAcceleration() {
        float acceleration = (GetTorqueOnWheel() * radius) / GetInertia();
        return acceleration;
    }

    float GetInertia() {
        float inertia = 0.5f * (mass * radius * radius); // Can be better
        return inertia;
    }
}
