package org.firstinspires.ftc.teamcode.Hardware;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// test comment from Christopher
public class RRHWProfile {
    public static final double STRAFE_FACTOR = 1;
    public static final double DRIVE_TICKS_PER_INCH = 44;
    public static final double DRIVE_TICKS_PER_INCH_ODO = 0.003220945;
    /* Public OpMode members. */
    public RevIMU imu = null;

    public Servo servoClawLeft = null;
    public Servo servoClawRight = null;
    public Servo servoWrist = null;

    public Servo servoArmLeft = null;
    public Servo servoArmRight = null;

    public Servo servoSlideLeft = null;
    public Servo servoSlideRight = null;
    public DcMotor perpOdo = null;
    public DcMotor parOdo = null;

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
    public void init(HardwareMap ahwMap, boolean driveMotors) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        if(driveMotors) {
            imu = new RevIMU(hwMap);
            imu.init();

            // Define and Initialize Motors
            motorLF = hwMap.get(DcMotor.class, "motorLF");
            motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLF.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorLF.setPower(0);


            motorLR = hwMap.get(DcMotor.class, "motorLR");
            motorLR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLR.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
            motorLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorLR.setPower(0);


            motorRF = hwMap.get(DcMotor.class, "motorRF");
            motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRF.setDirection(DcMotor.Direction.REVERSE);
            motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorRF.setPower(0);


            motorRR = hwMap.get(DcMotor.class, "motorRR");
            motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRR.setDirection(DcMotor.Direction.REVERSE);
            motorRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorRR.setPower(0);

            perpOdo = hwMap.get(DcMotor.class, "perp");
            perpOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            perpOdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            parOdo = hwMap.get(DcMotor.class, "motorClimbRight");
            parOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            parOdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        motorLift = hwMap.get(DcMotorEx.class, "motorLift");
        motorLift.setDirection(DcMotorEx.Direction.FORWARD);
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


    }
}  // end of HWProfile Class