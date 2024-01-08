package org.firstinspires.ftc.teamcode.Hardware;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

// test comment from Christopher
public class HWProfile {
    public static final double STRAFE_FACTOR = 1;
    public static final double BUCKET_RESET_POS = 1;
    public static final double BUCKET_SCORE_POS = .15;
    public static final double FOUR_BAR_OUT = .31;
    public static final double FOUR_BAR_IN = 1;
    public static final double PULLEY_SERVO_DROP = .25;
    public static final double PULLEY_SERVO_PICKUP = .05;
    public static final double PULLEY_SERVO_ZERO = 0;
    public static final double TURN_SPEED = .1;
    public static final int LIFT_MIN_POS = 0;
    public static final double FOUR_BAR_MID = .5;
    public static final double MAX_EXTEND = .5;
    public static final double MIN_EXTEND = 1;
    public static final double CLAW_OPEN = 1;
    public static final double CLAW_CLOSE = .55;
    /* Public OpMode members. */
    public MotorEx motorLeftFront   =   null;
    public MotorEx motorRightFront  =   null;
    public MotorEx motorLeftRear    =   null;
    public MotorEx motorRightRear   =   null;
    public DcMotor climbMotorRight   =   null;
    public DcMotor climbMotorLeft    =   null;
//    public DcMotorEx winchMotor    =   null;

//    public DcMotorEx lamp   =   null;

    public RevIMU imu =                 null;

    public DcMotor motorLF   = null;
    public DcMotor  motorLR  = null;
    public DcMotor  motorRF     = null;
    public DcMotor  motorRR    = null;

    public Servo fourBarLF = null;
    public Servo pulleyServo;
    public Servo fourBarRF = null;

    public Servo bucketServo1 = null;

    public Servo RFServoExtend = null;
    public Servo RFClaw = null;
    public Servo LFClaw = null;
    public Servo LFServoExtend = null;
    //    public DcMotor motorLeftLift = null;
//    public DcMotor motorRightLift = null;
//    public BNO055IMU imu = null;

    public DcMotorEx slidesMotor = null;
    //    public final DistanceSensor armSensor = null;


    public final double DRIVE_TICKS_PER_INCH = 44;      //temporary values => To be updated
    public final int LIFT_MAX_POS = 2310;



    /* local OpMode members. */
    HardwareMap hwMap           =  null;

    /* Constructor */
    public HWProfile(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define Motors utilizing FTCLib class
        motorLeftFront = new MotorEx(hwMap, "motorLF", Motor.GoBILDA.RPM_435);
        motorLeftFront.setRunMode(Motor.RunMode.RawPower);
        motorLeftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorLeftFront.setInverted(false);
        motorLeftFront.resetEncoder();

        motorLeftRear = new MotorEx(hwMap, "motorLR", Motor.GoBILDA.RPM_435);
        motorLeftRear.setRunMode(Motor.RunMode.RawPower);
        motorLeftRear.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorLeftRear.setInverted(false);
        motorLeftRear.resetEncoder();

        motorRightFront = new MotorEx(hwMap, "motorRF", Motor.GoBILDA.RPM_435);
        motorRightFront.setRunMode(Motor.RunMode.RawPower);
        motorRightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorRightFront.setInverted(true);
        motorRightFront.resetEncoder();

        motorRightRear = new MotorEx(hwMap, "motorRR", Motor.GoBILDA.RPM_435);
        motorRightRear.setRunMode(Motor.RunMode.RawPower);
        motorRightRear.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorRightRear.setInverted(false);
        motorRightRear.resetEncoder();




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

//        lamp = hwMap.get(DcMotorEx.class, "lamp");
//        lamp.setPower(0);

//        climbMotorLeft = hwMap.get(DcMotorEx.class, "motorLeftLift");
//        climbMotorLeft.setDirection(DcMotorEx.Direction.FORWARD);
//        climbMotorLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
////        climbMotorLeft.setTargetPosition(0);
////        climbMotorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        climbMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        climbMotorLeft.setPower(0);               // set motor power
//
//        climbMotorRight = hwMap.get(DcMotor.class, "motorRightLift");
//        climbMotorRight.setDirection(DcMotorEx.Direction.REVERSE);
//        climbMotorRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
////        climbMotorRight.setTargetPosition(0);
////        climbMotorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        climbMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        climbMotorRight.setPower(0);               // set motor power

        slidesMotor = hwMap.get(DcMotorEx.class, "mtr arm");
        slidesMotor.setDirection(DcMotorEx.Direction.FORWARD);
        slidesMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setTargetPosition(0);
        slidesMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesMotor.setPower(0);               // set motor power

//        winchMotor = hwMap.get(DcMotorEx.class, "winchMotor");
//        winchMotor.setDirection(DcMotorEx.Direction.REVERSE);
//        winchMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        winchMotor.setTargetPosition(0);
//        winchMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        winchMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        bucketServo1 = hwMap.get(Servo.class, "slide servo R");
        fourBarLF = hwMap.get(Servo.class, "LF 4B srv");
        fourBarRF = hwMap.get(Servo.class, "RF 4B srv");
        pulleyServo = hwMap.get(Servo.class, "LF wrist srv");
        RFServoExtend = hwMap.get(Servo.class, "RF srv extend");
        LFServoExtend = hwMap.get(Servo.class, "LF srv extend");
        RFClaw = hwMap.get(Servo.class, "RF claw srv");
        LFClaw = hwMap.get(Servo.class, "LF claw srv");

        // imu init
        imu = new RevIMU(hwMap);
        imu.init();

        /*
        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
         */

    }
}  // end of HWProfile Class
