using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class VehicleController : MonoBehaviour
{
    public CarBody carBody;
    public Engine engine;
    public Gearbox gearbox;
    Suspension suspension;

    AudioSource audioSource;

    public float corneringStiffness;
    public float velocityForward, velocityLateral, rpm;
    public int currentGear;

    // Wheels
    public Wheel[] wheels;  // FL FR RL RR
    public float brakeForce;
    public float throttle, steering, brake;
    public float maxSteerAngle = 30;

    public float magicValue;
    
    public float rearSlipAngle, frontSlipAngle;
    float sn;
	float cs;
    public Vector2 velocity;
    float yawspeed;
    float rot_angle;
    public float sideslip;
    Vector2 flatf;
    Vector2 flatr;
    bool front_slip, rear_slip;
    Vector2 ftraction;
    Vector2 resistance;
    public Vector2 force;
    float torque;
    Vector2 acceleration;
    float angular_acceleration;
    float angularVelocity;
    public Vector2 acceleration_wc, carVelocity_wc, carPos_wc;
    float carAngle;
    float wheelAngularVelocity;

    public float minFrictionCircleValue;
    public AnimationCurve wheelGrip;

    // Models
    [SerializeField] Transform carBodyHolder;

    // UI
    public Text rpmText, speedText, gearText;

    void Start()
    {
        suspension = GetComponent<Suspension>();
        audioSource = GetComponent<AudioSource>();
        GetComponent<Rigidbody>().detectCollisions = true;
    }

    float timer;
    void Update()
    {
        if(Application.isFocused) {
            throttle = (1 + Input.GetAxis("Throttle")) / 2;
            brake = (1 + Input.GetAxis("Brake")) / 2;

            steering = Input.GetAxis("Steering");
            if(Input.GetButtonDown("GearUp")) {
                if(currentGear < gearbox.gearRatios.Length - 1) {
                    currentGear++;
                }
            }
            if(Input.GetButtonDown("GearDown")) {
                if(currentGear > 0) {
                    currentGear--;
                }
            }
        }

        // Physics
        DetectGroundHeight();
        CarPhysics();

        // Visuals
        VisualWeightTransfer();
        VisualWheelModelRotation();

        // Audio
        audioSource.pitch = 0.3f + rpm * 0.0003f;

        // UI
        rpmText.text = rpm.ToString("F0");
        speedText.text = (velocity.x * 3.6f).ToString("F0");
        gearText.text = (currentGear+1).ToString();

        timer += Time.deltaTime;
    }


    void CarPhysics() {
        rpm = gearbox.GetRPM();
        sn = Mathf.Sin(carAngle);
        cs = Mathf.Cos(carAngle);

        float steeringAngleDeg = steering * maxSteerAngle;
        float steeringAngleRad = steeringAngleDeg / 180 * Mathf.PI;

        // Velocity from world space to car 
        velocity.x =  cs * carVelocity_wc.y + sn * carVelocity_wc.x;
        velocity.y = -sn * carVelocity_wc.y + cs * carVelocity_wc.x;

    // Lateral force on wheels
    //	
        yawspeed = carBody.wheelBase * 0.5f * angularVelocity;	

        if(velocity.x == 0) rot_angle = 0;
        else rot_angle = Mathf.Atan2( yawspeed, Mathf.Abs(velocity.x));

        sideslip = Mathf.Atan2( velocity.y, Mathf.Abs(velocity.x));

        // Calculate slip angles for front and rear wheels
        frontSlipAngle = sideslip + rot_angle - (Mathf.Sign(velocity.x) * steeringAngleRad);
        rearSlipAngle  = sideslip - rot_angle;

        float weightFront = 0;
        float weightRear = 0;
        float corneringStiffnessFront = 0;
        float corneringStiffnessRear = 0;
        float forwardGripFront = 0;
        float forwardGripRear = 0;
        foreach(Wheel w in wheels) {
            if(w.isGrounded) {
                if(w.fl || w.fr) {
                    weightFront += w.GetWeightOnWheel();
                    corneringStiffnessFront += wheelGrip.Evaluate(Mathf.Abs(frontSlipAngle*180f/Mathf.PI));
                    forwardGripFront += w.coefficientFrictionBySlipRatio.Evaluate(w.GetSlipRatio());
                }
                if(w.rl || w.rr) {
                    weightRear += w.GetWeightOnWheel();
                    corneringStiffnessRear += wheelGrip.Evaluate(Mathf.Abs(rearSlipAngle*180f/Mathf.PI));
                    forwardGripRear += w.coefficientFrictionBySlipRatio.Evaluate(w.GetSlipRatio());
                }
            }
        }

        // longtitudinal force
        float frontWheelsTractionForceForward = 0;
        float rearWheelsTractionForceForward = 0;
        float frontWheelsTractionForceForwardMax = 0;
        float rearWheelsTractionForceForwardMax = 0;
        foreach(Wheel w in wheels) {
            if(w.rl || w.rr) {
                float tractionForce = w.GetWheelForwardTractionForce();               
                float maxTractionForce = forwardGripRear * w.GetWeightOnWheel();
                if(tractionForce > maxTractionForce) {
                    tractionForce = maxTractionForce;
                }
                if(w.GetSlipRatio() > 1f) {
                    if(!w.audioSource.isPlaying) w.audioSource.Play();
                }
                else {
                    if(w.audioSource.isPlaying) w.audioSource.Stop();
                }
                rearWheelsTractionForceForwardMax += maxTractionForce;
                rearWheelsTractionForceForward += tractionForce;          
            }
            else {
                float tractionForce = w.GetWheelForwardTractionForce();               
                float maxTractionForce = forwardGripFront * w.GetWeightOnWheel();

                if(tractionForce > maxTractionForce) {
                    tractionForce = maxTractionForce;
                }
                frontWheelsTractionForceForwardMax += maxTractionForce;
                frontWheelsTractionForceForward += tractionForce;
            } 
        }

        ftraction.x = rearWheelsTractionForceForward + frontWheelsTractionForceForward;
        ftraction.y = 0;

        
        // Forces and torque on body
        // Friction circle thingy: sqrt(1^2 - (tractionForce / maxTractionForce)^2)
        float frictionCircleMultiplierFront = Mathf.Sqrt(1f - (frontWheelsTractionForceForward / frontWheelsTractionForceForwardMax) * (frontWheelsTractionForceForward / frontWheelsTractionForceForwardMax));
        float frictionCircleMultiplierRear = Mathf.Sqrt(1f - (rearWheelsTractionForceForward / rearWheelsTractionForceForwardMax) * (rearWheelsTractionForceForward / rearWheelsTractionForceForwardMax));
        if(float.IsNaN(frictionCircleMultiplierFront) || frictionCircleMultiplierFront < minFrictionCircleValue) {
            frictionCircleMultiplierFront = minFrictionCircleValue;
        }
        if(float.IsNaN(frictionCircleMultiplierRear) || frictionCircleMultiplierRear < minFrictionCircleValue) {
            frictionCircleMultiplierRear = minFrictionCircleValue;
        }

        if(Mathf.Abs(velocity.x) < 1) {
            frictionCircleMultiplierFront = 1;
            frictionCircleMultiplierRear = 1;
        }

        flatf.y = -Mathf.Sign(frontSlipAngle) * corneringStiffnessFront * weightFront * frictionCircleMultiplierFront;         

        flatr.y = -Mathf.Sign(rearSlipAngle) * corneringStiffnessRear * weightRear * frictionCircleMultiplierRear; 
        
  
        float totalRollingResitance = 0;
        foreach(Wheel w in wheels) {
            totalRollingResitance += w.GetRollingResistance();
        }
        
        // Resistances
        resistance.x = - (totalRollingResitance + carBody.GetAeroDrag());
        if(velocity.magnitude > 3) {
            resistance.y = - Mathf.Sign(velocity.y) * (carBody.GetAeroDragSide() + totalRollingResitance);
        }


        // Forces
        force.x = ftraction.x + resistance.x;
        force.y = ftraction.y + Mathf.Cos(steeringAngleRad) * flatf.y + flatr.y + resistance.y;
         

        // Torque
        torque = carBody.fromCenterToFrontAxle * flatf.y - carBody.fromCenterToRearAxle * flatr.y;



    // Acceleration
        acceleration.x = force.x/carBody.mass;
        acceleration.y = force.y/carBody.mass;
        
        angular_acceleration = torque / carBody.mass;

    // Velocity and position
        // Acceleration from world space to car
        acceleration_wc.x = cs * acceleration.y + sn * acceleration.x;
        acceleration_wc.y = -sn * acceleration.y + cs * acceleration.x;

        // Velocity
        carVelocity_wc.x += Time.deltaTime * acceleration_wc.x;
        carVelocity_wc.y += Time.deltaTime * acceleration_wc.y;

        // Position
        carPos_wc.x += Time.deltaTime * carVelocity_wc.x;
        carPos_wc.y += Time.deltaTime * carVelocity_wc.y;
        if(float.IsNaN(carPos_wc.x)) {
            carPos_wc.x = 0;
        }
        if(float.IsNaN(carPos_wc.y)) {
            carPos_wc.y = 0;
        }

    // Angular velocity
        angularVelocity += Time.deltaTime * angular_acceleration;

        carAngle += Time.deltaTime * angularVelocity;

        float carAngleDeg = carAngle * 180 / Mathf.PI;

        transform.position = new Vector3(carPos_wc.x, transform.position.y, carPos_wc.y);
        transform.eulerAngles = new Vector3(transform.eulerAngles.x, carAngleDeg, transform.eulerAngles.z);
    }

    public float GetVelocityMagnitude() {
        float magnitude = Mathf.Sqrt(velocity.x * velocity.x + velocity.y * velocity.y);
        return magnitude;
    }

    void DetectGroundHeight() {
        RaycastHit[] hit = new RaycastHit[4];
        int i = 0;
        foreach(Wheel w in wheels) {
            if (Physics.Raycast(w.model.position + new Vector3(0, 0.3f, 0), transform.TransformDirection(-Vector3.up), out hit[i], Mathf.Infinity, 7))
            {
                Debug.DrawRay(w.model.position, transform.TransformDirection(-Vector3.up) * hit[i].distance, Color.yellow);
                if(w.model.position.y - w.radius - 0.08f <= hit[i].point.y) { // 0.08f is offset, can be removed when suspension is developed
                    w.isGrounded = true;
                }
                else {
                    w.isGrounded = false;
                }
            }
            else
            {
                Debug.DrawRay(w.model.position, transform.TransformDirection(-Vector3.up) * 1000, Color.white);

                w.isGrounded = true;
            }
            i++;
        }

        CalculateCarRotationToMatchGround(hit);
    }

    float yPos;
    float yVelocity;
    bool isGrounded;
    float rotationX;
    float rotationZ;
    float frontHeight;
    float rearHeight;

    void CalculateCarRotationToMatchGround(RaycastHit[] hit) {
        if(wheels[0].isGrounded || wheels[1].isGrounded || wheels[2].isGrounded || wheels[3].isGrounded) {
            //isGrounded = true;
        }
        else {
            //isGrounded = false;
        }


        float newGroundHeight = (hit[0].point.y + hit[1].point.y + hit[2].point.y + hit[3].point.y) / 4f;
        if(yPos <= newGroundHeight) {
            yVelocity = (newGroundHeight - yPos) / Time.deltaTime;
        }
        else{
            yVelocity -= (Environment.gravity * magicValue) * Time.deltaTime;
        }
        
        yPos += yVelocity * Time.deltaTime;
                
        // No underground here
        if(yPos <= newGroundHeight) { 
            //yPos = newGroundHeight;     
        }

    if(isGrounded)
    {
        // Forward rotation
        frontHeight = (hit[0].point.y + hit[1].point.y) / 2f;
        rearHeight = (hit[2].point.y + hit[3].point.y) / 2f;

        float frontAndRearDifference = frontHeight - rearHeight;
        if(frontAndRearDifference != 0) {
            float fromFrontToMiddle = carBody.wheelBase / 2f;
            float middleHeight = (frontHeight + rearHeight) / 2f;
            float alpha = Mathf.Atan(fromFrontToMiddle / middleHeight);
            float c = (Mathf.Sqrt(carBody.wheelBase * carBody.wheelBase + frontAndRearDifference * frontAndRearDifference)) / 2;
            float gamma = Mathf.Sign(frontAndRearDifference) * (0.5f * Mathf.PI) - (0.5f * Mathf.PI) + Mathf.Asin(fromFrontToMiddle / (Mathf.Sign(frontAndRearDifference) * c));
            float delta = Mathf.Atan(middleHeight / fromFrontToMiddle);
            float epsilon = (0.5f * Mathf.PI) - delta;
            float beta = Mathf.PI - gamma - epsilon;
            rotationX = ((0.5f * Mathf.PI) - (alpha + beta))  * 180f / Mathf.PI;
        }
        else {
            rotationX = 0;
        }

        // Sideways rotation
        float leftHeight = (hit[0].point.y + hit[2].point.y) / 2f;
        float rightHeight = (hit[1].point.y + hit[3].point.y) / 2f;

        float leftAndRightDifference = leftHeight - rightHeight;
        if(leftAndRightDifference != 0) {
            float fromLeftToMiddle = carBody.fromLeftWheelToRight / 2f;
            float middleHeight = (leftHeight + rightHeight) / 2f;
            float alpha = Mathf.Atan(fromLeftToMiddle / middleHeight);
            float c = (Mathf.Sqrt(carBody.fromLeftWheelToRight * carBody.fromLeftWheelToRight + leftAndRightDifference * leftAndRightDifference)) / 2;
            float gamma = Mathf.Sign(leftAndRightDifference) * (0.5f * Mathf.PI) - (0.5f * Mathf.PI) + Mathf.Asin(fromLeftToMiddle / (Mathf.Sign(leftAndRightDifference) * c));
            float delta = Mathf.Atan(middleHeight / fromLeftToMiddle);
            float epsilon = (0.5f * Mathf.PI) - delta;
            float beta = Mathf.PI - gamma - epsilon;
            rotationZ = ((0.5f * Mathf.PI) - (alpha + beta))  * 180f / Mathf.PI;
        }
        else {
            rotationZ = 0;
        }
    }

        transform.eulerAngles = new Vector3(rotationX, transform.eulerAngles.y, rotationZ);
        transform.position = new Vector3(transform.position.x, yPos, transform.position.z);
    }

    void OnCollisionStay(Collision col) {
        isGrounded = true;
    }

    void OnCollisionExit(Collision col) {
        isGrounded = false;
        Debug.Log("Exit col");
    }


    /* Visuals */

    void VisualWheelModelRotation() {
        foreach(Wheel w in wheels) {  
            if(w.fl || w.fr) {
                w.rotationLast += w.GetAngularVelocity() / (2*Mathf.PI);
                Vector3 temp = w.model.localEulerAngles;
                temp.y = maxSteerAngle * steering;
                temp.z = 0;
                temp.x = w.rotationLast;
                w.model.localEulerAngles = temp;

            }
            else {
                w.model.Rotate(w.GetAngularVelocity() / (2*Mathf.PI), 0, 0);
            }
        }
    }

    void VisualWeightTransfer() {

        // Forward

        float frontHeight = suspension.springLength - suspension.GetSpringPosFront();
        float rearHeight = suspension.springLength - suspension.GetSpringPosRear();

        float hypotenuseX = Mathf.Sqrt(carBody.wheelBase * carBody.wheelBase + rearHeight * rearHeight);
        
        float rotationX = Mathf.Asin((rearHeight-frontHeight)/hypotenuseX) * 180f / Mathf.PI;

        // Lateral

        float rotationZ = 0;
        float leftHeight = suspension.springLength - suspension.GetSpringPosLeft();
        float rightHeight = suspension.springLength - suspension.GetSpringPosRight();

        float leftAndRightDifference = leftHeight - rightHeight;
        if(leftAndRightDifference != 0) {
            float fromLeftToMiddle = carBody.fromLeftWheelToRight / 2f;
            float middleHeight = (leftHeight + rightHeight) / 2f;
            float alpha = Mathf.Atan(fromLeftToMiddle / middleHeight);
            float c = (Mathf.Sqrt(carBody.fromLeftWheelToRight * carBody.fromLeftWheelToRight + leftAndRightDifference * leftAndRightDifference)) / 2;
            float gamma = Mathf.Sign(leftAndRightDifference) * (0.5f * Mathf.PI) - (0.5f * Mathf.PI) + Mathf.Asin(fromLeftToMiddle / (Mathf.Sign(leftAndRightDifference) * c));
            float delta = Mathf.Atan(middleHeight / fromLeftToMiddle);
            float epsilon = (0.5f * Mathf.PI) - delta;
            float beta = Mathf.PI - gamma - epsilon;
            rotationZ = ((0.5f * Mathf.PI) - (alpha + beta))  * 180f / Mathf.PI; // 
        }


        carBodyHolder.localPosition = new Vector3(carBodyHolder.localPosition.x, rearHeight, carBodyHolder.localPosition.z);
        carBodyHolder.localEulerAngles = new Vector3(rotationX, 0, rotationZ); 
    }

}