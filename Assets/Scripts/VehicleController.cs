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

    public float corneringStiffness;
    public float velocityForward, velocityLateral, rpm;
    public int currentGear;
    float  xPos, yPos;

    // Wheels
    public Wheel[] wheels;  // FL FR RL RR
    public float brakeForce;
    public float throttle, steering, brake;
    public float maxSteerAngle = 40;

    public float maxGrip;

    public float magicValue;
    
    float rearSlipAngle, frontSlipAngle;
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

    public AnimationCurve wheelGrip;

    // Models
    [SerializeField] Transform carBodyHolder;

    // UI
    public Text rpmText, speedText, gearText;

    void Start()
    {
        suspension = GetComponent<Suspension>();
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
        Physics();

        // Visuals
        VisualWeightTransfer();

        // UI
        rpmText.text = rpm.ToString("F0");
        speedText.text = (velocity.x * 3.6f).ToString("F0");
        gearText.text = (currentGear+1).ToString();

        timer += Time.deltaTime;
    }


    void Physics() {
        rpm = gearbox.GetRPM();
        sn = Mathf.Sin(carAngle);
        cs = Mathf.Cos(carAngle);

        float steeringAngleDeg = steering * maxSteerAngle;
        float steeringAngleRad = steeringAngleDeg / 180 * Mathf.PI;

        if( steeringAngleRad != 0.0f )
        {
            //int breakme = 1;
        }

        // SAE convention: x is to the front of the car, y is to the right, z is down

        //	bangz: Velocity of Car. Vlat and Vlong
        // transform velocity in world reference frame to velocity in car reference frame
        velocity.x =  cs * carVelocity_wc.y + sn * carVelocity_wc.x;
        velocity.y = -sn * carVelocity_wc.y + cs * carVelocity_wc.x;

    // Lateral force on wheels
    //	
        // Resulting velocity of the wheels as result of the yaw rate of the car body
        // v = yawrate * r where r is distance of wheel to CG (approx. half wheel base)
        // yawrate (ang.velocity) must be in rad/s
        //
        yawspeed = carBody.wheelBase * 0.5f * angularVelocity;	

        //bangz: velocity.x = fVLong_, velocity.y = fVLat_
        if( velocity.x == 0 )		// TODO: fix singularity
            rot_angle = 0;
        else
            rot_angle = Mathf.Atan2( yawspeed, Mathf.Abs(velocity.x));

        

        // Calculate the side slip angle of the car (a.k.a. beta)
        //if( velocity.x == 0 )		// TODO: fix singularity
        //    sideslip = 0;
        //else
            sideslip = Mathf.Atan2( velocity.y, Mathf.Abs(velocity.x));
            /*if(velocity.y < 0) {
                sideslip = -sideslip;
            }*/

        // Calculate slip angles for front and rear wheels (a.k.a. alpha)
        frontSlipAngle = sideslip + rot_angle - (Mathf.Sign(velocity.x) * steeringAngleRad);
        rearSlipAngle  = sideslip - rot_angle;



        // weight per axle = half car mass times 1G (=9.8m/s^2) /////////////////////////////////////////////////////////////////////////////////
        float weightFront = 0;
        float weightRear = 0;
        float corneringStiffnessFront = 0;
        float corneringStiffnessRear = 0;
        foreach(Wheel w in wheels) {
            if(w.fl || w.fr) {
                weightFront += w.GetWeightOnWheel();
                corneringStiffnessFront += w.corneringStiffness;
            }
            if(w.rl || w.rr) {
                weightRear += w.GetWeightOnWheel();
                corneringStiffnessRear += w.corneringStiffness;
            }
        }

        // lateral force on front wheels = (Ca * slip angle) capped to friction circle * load
        flatf.x = 0;
        //float maxLateralForceFront = magicValue * Mathf.Sign(frontSlipAngle) * wheelGrip.Evaluate(Mathf.Abs(frontSlipAngle*180f/Mathf.PI)) * weightFront;
        flatf.y = magicValue * Mathf.Sign(frontSlipAngle) * wheelGrip.Evaluate(Mathf.Abs(frontSlipAngle*180f/Mathf.PI)) * weightFront; //
        //flatf.y = corneringStiffnessFront * frontSlipAngle;
        //flatf.y = Mathf.Min(maxGrip, flatf.y);
        //flatf.y = Mathf.Max(-maxGrip, flatf.y);
        //flatf.y *= weightFront;
        if(front_slip)
            flatf.y *= 0.5f;

        // lateral force on rear wheels
        flatr.x = 0;
        flatr.y = magicValue * Mathf.Sign(rearSlipAngle) * wheelGrip.Evaluate(Mathf.Abs(rearSlipAngle*180f/Mathf.PI)) * weightRear;
        //flatr.y = corneringStiffnessRear * rearSlipAngle;
        //flatr.y = Mathf.Min(maxGrip, flatr.y);
        //flatr.y = Mathf.Max(-maxGrip, flatr.y);
        //flatr.y *= weightRear;
        if(rear_slip)
            flatr.y *= 0.5f;

        // longtitudinal force on rear wheels - very simple traction model
        float wheelsTractionForceForward = 0;
        foreach(Wheel w in wheels) {
            if(w.rl || w.rr) {
                float tractionForce = w.GetWheelForwardTractionForce();               
                float maxTractionForce = w.coefficientOfFrictionForward * w.GetWeightOnWheel();
                if(tractionForce > maxTractionForce) {
                    tractionForce = maxTractionForce;
                    if(w.rl) Debug.Log("LEFTI RUAPII");
                    if(w.rr) Debug.Log("Raitti RUAPII");
                } 
                wheelsTractionForceForward += tractionForce;
            }
        }
        ftraction.x = wheelsTractionForceForward;
        ftraction.x -= brakeForce * brake * Mathf.Sign(velocity.x);
        ftraction.y = 0;
        if(rear_slip)
            ftraction.x *= 0.5f;

    // Forces and torque on body
        float totalRollingResitance = 0;
        foreach(Wheel w in wheels) {
            totalRollingResitance += w.GetRollingResistance();
        }
        // drag and rolling resistance
        resistance.x = - (totalRollingResitance + carBody.GetAeroDrag());
        resistance.y = - (carBody.GetAeroDragSide() + (totalRollingResitance * Mathf.Sign(velocity.y)));

        // sum forces
        force.x = ftraction.x + Mathf.Sin(steeringAngleRad) * flatf.x + flatr.x + resistance.x; //
        force.y = ftraction.y + Mathf.Cos(steeringAngleRad) * flatf.y + flatr.y + resistance.y;	//



        // torque on body from lateral forces
        torque = carBody.fromCenterToFrontAxle * flatf.y - carBody.fromCenterToRearAxle * flatr.y;

    // Acceleration
        
        // Newton F = m.a, therefore a = F/m
        acceleration.x = force.x/carBody.mass;
        acceleration.y = force.y/carBody.mass;
        
        angular_acceleration = torque / carBody.mass;

    // Velocity and position
        
        // transform acceleration from car reference frame to world reference frame
        acceleration_wc.x = cs * acceleration.y + sn * acceleration.x;
        acceleration_wc.y = -sn * acceleration.y + cs * acceleration.x;

        // velocity is integrated acceleration
        //
        carVelocity_wc.x += Time.deltaTime * acceleration_wc.x;
        carVelocity_wc.y += Time.deltaTime * acceleration_wc.y;

        // position is integrated velocity
        //
        carPos_wc.x += Time.deltaTime * carVelocity_wc.x;
        carPos_wc.y += Time.deltaTime * carVelocity_wc.y;


    // Angular velocity and heading

        // integrate angular acceleration to get angular velocity
        //
        angularVelocity += Time.deltaTime * angular_acceleration;

        // integrate angular velocity to get angular orientation
        //
        carAngle += Time.deltaTime * angularVelocity;
        //carAngle = carAngle * 180 / Mathf.PI;
        float carAngleDeg = carAngle * 180 / Mathf.PI;

        transform.position = new Vector3(carPos_wc.x, 0, carPos_wc.y);
        transform.eulerAngles = new Vector3(0, carAngleDeg, 0);
    }

    public float GetVelocityMagnitude() {
        float magnitude = Mathf.Sqrt(velocity.x * velocity.x + velocity.y * velocity.y);
        return magnitude;
    }

    void VisualWeightTransfer() {

        // Forward

        float frontHeight = suspension.springLength - suspension.GetSpringPosFront();
        float rearHeight = suspension.springLength - suspension.GetSpringPosRear();

        float hypotenuseX = Mathf.Sqrt(carBody.wheelBase * carBody.wheelBase + rearHeight * rearHeight);
        
        float rotationX = Mathf.Asin((rearHeight-frontHeight)/hypotenuseX) * 180f / Mathf.PI;
        rotationX = 0;

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
