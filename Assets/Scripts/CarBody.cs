using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarBody : MonoBehaviour
{
    public VehicleController vehicleController;

    public float mass;
    public float wheelBase = 2.268f;
    public float centerOfGravityHeight = 0.4f;
    public float fromCenterToFrontAxle = 1.268f;
    public float fromCenterToRearAxle = 1f;
    public float fromLeftWheelToRight = 1.8f;

    public float aeroDragCoefficient = 0.5f;
    public float aeroDragArea = 1.5f;
    public float aeroDragAreaSide = 2.5f;

    public float GetAeroDrag() {
        float drag = Mathf.Sign(vehicleController.velocity.x) * (aeroDragCoefficient * (0.5f * Environment.densityOfAir * aeroDragArea * vehicleController.velocity.x * vehicleController.velocity.x));
        return drag;
    }

    public float GetAeroDragSide() {
        float drag = Mathf.Sign(vehicleController.velocity.x) * (Mathf.Sign(vehicleController.velocity.y) * (aeroDragCoefficient * (0.5f * Environment.densityOfAir * aeroDragAreaSide * vehicleController.velocity.y * vehicleController.velocity.y)));
        return drag;
    }
}
