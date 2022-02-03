using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FFBController : MonoBehaviour
{
    VehicleController vehicleController;
    public int magic1, magic2, magic3, magic4;
    public int dambStrength, constForceStrength;
    public float forceThreshold;
    void Start()
    {
        vehicleController = GetComponent<VehicleController>();
        Debug.Log("SteeringInit:" + LogitechGSDK.LogiSteeringInitialize(false));
    }

    void OnApplicationQuit()
    {
        Debug.Log("SteeringShutdown:" + LogitechGSDK.LogiSteeringShutdown());
    }


    bool forceToRight, forceToLeft;
    void Update()
    {
        if (LogitechGSDK.LogiUpdate() && LogitechGSDK.LogiIsConnected(0))
        {
            if(vehicleController.velocity.magnitude > 3) {
                if(vehicleController.frontSlipAngle > forceThreshold && !forceToRight) {
                    LogitechGSDK.LogiPlayConstantForce(0, constForceStrength);
                    forceToRight = true;
                    forceToLeft = false;
                    if (LogitechGSDK.LogiIsPlaying(0, LogitechGSDK.LOGI_FORCE_DAMPER))
                    {
                        LogitechGSDK.LogiStopDamperForce(0);
                    }
                }
                if(vehicleController.frontSlipAngle < -forceThreshold && !forceToLeft) {
                    LogitechGSDK.LogiPlayConstantForce(0, -constForceStrength);
                    forceToRight = false;
                    forceToLeft = true;
                    if (LogitechGSDK.LogiIsPlaying(0, LogitechGSDK.LOGI_FORCE_DAMPER))
                    {
                        LogitechGSDK.LogiStopDamperForce(0);
                    }
                }
            }
            

            if(Mathf.Abs(vehicleController.frontSlipAngle) < forceThreshold) {
                if(!LogitechGSDK.LogiIsPlaying(0, LogitechGSDK.LOGI_FORCE_DAMPER)) {
                    LogitechGSDK.LogiPlayDamperForce(0, dambStrength);     
                }
                if (LogitechGSDK.LogiIsPlaying(0, LogitechGSDK.LOGI_FORCE_CONSTANT)) {
                    LogitechGSDK.LogiStopConstantForce(0);
                }
                forceToRight = false;
                forceToLeft = false;
            }
        }
    }
}
