package org.firstinspires.ftc.teamcode.Hardware;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

// test comment from Christopher
public class RRHWProfile {
    public static final double STRAFE_FACTOR = 1;
    public double oldKP = 0;
    public static final double DRIVE_TICKS_PER_INCH = 44;
    public static final double DRIVE_TICKS_PER_INCH_ODO = 0.003220945;
    /* Public OpMode members. */
    public RevIMU imu = null;

    public Servo servoClawLeft = null;
    public Servo servoClawRight = null;
    public Servo servoWrist = null;

    public Servo servoArmLeft = null;
    public Servo servoArmRight = null;
    public Servo fingerServo = null;

    public Servo servoSlideLeft = null;
    public Servo servoSlideRight = null;
    public Servo servoBucketFingerLeft = null;
    public Servo servoBucketFingerRight = null;
    public DcMotor perpOdo = null;
    public DcMotor parOdo = null;

    public Servo servoDrone = null;
    public Servo servoBucket = null;

    public DcMotorEx motorLift = null;
    public DcMotorEx secondMotorLift = null;

    public Motor FTCLIB_motorLift = null;
    public Motor FTCLIB_secondMotorLift = null;
    public DcMotor droneActuator = null;
    public DcMotor motorLF   = null;
    public DcMotor  motorLR  = null;
    public MotorGroup liftMotorGroup = null;
    public DcMotor  motorRF     = null;
    public DcMotor  motorRR    = null;
    public LED ledThree = null;
    public LED ledFour = null;
    public LED ledOne = null;
    public LED ledTwo = null;

    public RevColorSensorV3 clawSensorRight;
    public RevColorSensorV3 clawSensorLeft;

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

        FTCLIB_motorLift = new Motor(hwMap, "motorLift", Motor.GoBILDA.RPM_223);
        FTCLIB_secondMotorLift = new Motor(hwMap, "secondSlideMotor", Motor.GoBILDA.RPM_223);
        liftMotorGroup = new MotorGroup(FTCLIB_motorLift, FTCLIB_secondMotorLift);

        liftMotorGroup.stopAndResetEncoder();
        FTCLIB_motorLift.stopAndResetEncoder();
        FTCLIB_secondMotorLift.stopAndResetEncoder();

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

            parOdo = hwMap.get(DcMotor.class, "droneActuator");
            parOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            parOdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            droneActuator = hwMap.get(DcMotorEx.class, "droneActuator");
            droneActuator.setDirection(DcMotorEx.Direction.FORWARD);
            droneActuator.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            droneActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            droneActuator.setPower(0);
        }

        motorLift = hwMap.get(DcMotorEx.class, "motorLift");
        motorLift.setDirection(DcMotorEx.Direction.REVERSE);
        motorLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setTargetPosition(0);
        motorLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setPower(0);               // set motor power

        secondMotorLift = hwMap.get(DcMotorEx.class, "secondSlideMotor");
        secondMotorLift.setDirection(DcMotorEx.Direction.FORWARD);
        secondMotorLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        secondMotorLift.setTargetPosition(0);
        secondMotorLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        secondMotorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        secondMotorLift.setPower(0);               // set motor power
        oldKP = secondMotorLift.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p;
        // set motor power

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        servoClawLeft = hwMap.get(Servo.class, "servoClawLeft");
        servoClawRight = hwMap.get(Servo.class, "servoClawRight");
        servoSlideLeft = hwMap.get(Servo.class, "servoSlideLeft");
        servoSlideRight = hwMap.get(Servo.class, "servoSlideRight");
        servoArmLeft = hwMap.get(Servo.class, "servoArmLeft");
        servoArmRight = hwMap.get(Servo.class, "servoArmRight");
        servoBucketFingerLeft = hwMap.get(Servo.class, "bucketServoLeft");
        servoBucketFingerRight = hwMap.get(Servo.class, "bucketServoRight");
        servoWrist = hwMap.get(Servo.class, "servoWrist");
        servoDrone = hwMap.get(Servo.class, "droneServo");
        servoBucket = hwMap.get(Servo.class, "servoBucket");
        fingerServo = hwMap.get(Servo.class, "fingerServo");

        clawSensorRight = hwMap.get(RevColorSensorV3.class, "clawSensorRight");
        clawSensorLeft = hwMap.get(RevColorSensorV3.class, "clawSensorLeft");

        ledOne = hwMap.get(LED.class, "led1");
        ledTwo = hwMap.get(LED.class, "led2");
        ledThree = hwMap.get(LED.class, "led3");
        ledFour = hwMap.get(LED.class, "led4");

        // init distance sensor
//        sensorLeft = hwMap.get(DistanceSensor.class, "sensorLeft");
//        sensorRight = hwMap.get(DistanceSensor.class, "sensorRight");


    }
}  // end of HWProfile Class