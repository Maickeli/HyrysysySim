using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Engine : MonoBehaviour
{
    public VehicleController vehicleController;
    public AnimationCurve torque; 
    public float rpmLimit;


    public float GetTorque() {
        if(vehicleController.rpm >= rpmLimit)
        {
            return 0;
        }
        else {
            return torque.Evaluate(vehicleController.rpm) * vehicleController.throttle;
        }
    }

}
