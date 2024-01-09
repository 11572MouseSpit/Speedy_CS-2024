package org.firstinspires.ftc.teamcode.Hardware;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// test comment from Christopher
public class RRHWProfile {
    /* Public OpMode members. */
    public RevIMU imu = null;

    public Servo servoClawLeft = null;
    public Servo servoClawRight = null;
    public Servo servoWrist = null;

    public Servo servoArmLeft = null;
    public Servo servoArmRight = null;

    public Servo servoSlideLeft = null;
    public Servo servoSlideRight = null;

    public Servo servoDrone = null;
    public Servo servoBucket = null;

    public DcMotorEx motorLift = null;
    public DcMotor motorLF   = null;
    public DcMotor  motorLR  = null;
    public DcMotor  motorRF     = null;
    public DcMotor  motorRR    = null;

    //    public final DistanceSensor armSensor = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;

    /* Constructor */
    public RRHWProfile(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        motorLift = hwMap.get(DcMotorEx.class, "motorLift");
        motorLift.setDirection(DcMotorEx.Direction.REVERSE);
        motorLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setTargetPosition(0);
        motorLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setPower(0);               // set motor power

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        servoClawLeft = hwMap.get(Servo.class, "servoClawLeft");
        servoClawRight = hwMap.get(Servo.class, "servoClawRight");
        servoSlideLeft = hwMap.get(Servo.class, "servoSlideLeft");
        servoSlideRight = hwMap.get(Servo.class, "servoSlideRight");
        servoArmLeft = hwMap.get(Servo.class, "servoArmLeft");
        servoArmRight = hwMap.get(Servo.class, "servoArmRight");
        servoWrist = hwMap.get(Servo.class, "servoWrist");
        servoDrone = hwMap.get(Servo.class, "servoDrone");
        servoBucket = hwMap.get(Servo.class, "servoBucket");

        // init distance sensor
//        sensorLeft = hwMap.get(DistanceSensor.class, "sensorLeft");
//        sensorRight = hwMap.get(DistanceSensor.class, "sensorRight");

        // imu init
        imu = new RevIMU(hwMap);
        imu.init();

    }
}  // end of HWProfile Class