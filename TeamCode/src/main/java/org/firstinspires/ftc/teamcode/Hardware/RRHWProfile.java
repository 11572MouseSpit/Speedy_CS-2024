package org.firstinspires.ftc.teamcode.Hardware;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

// test comment from Christopher
public class RRHWProfile {
    public static final double STRAFE_FACTOR = 1;
    public double oldKP = 0;
    public static final double DRIVE_TICKS_PER_INCH = 44;
    public static final double DRIVE_TICKS_PER_INCH_ODO = 0.003220945;
    /* Public OpMode members. */
    public IMU imu;
    public RevIMU imu2;

    public Servo servoClawLeft = null;
    public Servo servoClawRight = null;
    public Servo servoDrone = null;
    public Servo servoIntakeActuator = null;
    public Servo servoIntake = null;
    public Servo servoArmActuatorB = null;
    public Servo servoClawWrist = null;
    public Servo servoClimbLockLeft = null;
    public Servo servoClimbLockRight = null;



    /*
    public Servo servoWrist = null;

    public Servo servoArmLeft = null;
    public Servo servoArmRight = null;
    public Servo fingerServo = null;

    public Servo servoSlideLeft = null;
    public Servo servoSlideRight = null;
    public Servo servoBucketFingerLeft = null;
    public Servo servoBucketFingerRight = null;
    */
    public DcMotor perpOdo = null;
    public DcMotorEx motorLift = null;

    public DcMotorEx armMotor = null;
    public DcMotor parOdo = null;
    public DcMotorEx climbMotorLeft = null;
    public DcMotorEx climbMotorRight = null;

    /*

    public Servo servoBucket = null;

    public DcMotorEx secondMotorLift = null;

    */

    public Motor FTCLIB_motorLift = null;
    public Motor FTCLIB_secondMotorLift = null;
    public DcMotorEx motorLF   = null;
    public DcMotorEx  motorLR  = null;
    public MotorGroup liftMotorGroup = null;
    public DcMotorEx  motorRF     = null;
    public DcMotorEx  motorRR    = null;
    public Motor ftcLib_motorLF;
    public Motor ftcLib_motorLR;
    public Motor ftcLib_motorRF;
    public Motor ftcLib_motorRR;
    public RevColorSensorV3 clawSensorRight;
    public RevColorSensorV3 clawSensorLeft;
    public DigitalChannel leftLedBackRed;
    public DigitalChannel leftLedBackGreen;
    public DigitalChannel leftLedFrontRed;
    public DigitalChannel leftLedFrontGreen;
    public IMU.Parameters parameters;
    public DigitalChannel rightLedBackRed;
    public DigitalChannel rightLedBackGreen;
    public DigitalChannel rightLedFrontRed;
    public DigitalChannel rightLedFrontGreen;

    //    public final DistanceSensor armSensor = null;

    /* local OpMode members. */
    public HardwareMap hwMap           =  null;

    /* Constructor */
    public RRHWProfile(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, boolean driveMotors) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                logoFacingDirection, usbFacingDirection));

        imu = hwMap.get(IMU.class, "imu");

        imu.initialize(parameters);

        imu2 = new RevIMU(hwMap, "imu2");
        imu2.init();

        if(driveMotors) {
            // Define and Initialize Motors
            motorLF = hwMap.get(DcMotorEx.class, "motorLF");
            motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLF.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorLF.setPower(0);

            ftcLib_motorLF = new Motor(hwMap, "motorLF", Motor.GoBILDA.RPM_1150);
//            ftcLib_motorLF.setInverted(true);
            ftcLib_motorLF.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//            ftcLib_motorLF.setRunMode(Motor.RunMode.PositionControl);

            ftcLib_motorLR = new Motor(hwMap, "motorLR", Motor.GoBILDA.RPM_1150);
//            ftcLib_motorLR.setInverted(false);
            ftcLib_motorLR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//            ftcLib_motorLR.setRunMode(Motor.RunMode.PositionControl);

            ftcLib_motorRF = new Motor(hwMap, "motorRF", Motor.GoBILDA.RPM_1150);
//            ftcLib_motorRF.setInverted(true);
            ftcLib_motorRF.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//            ftcLib_motorRF.setRunMode(Motor.RunMode.PositionControl);

            ftcLib_motorRR = new Motor(hwMap, "motorRR", Motor.GoBILDA.RPM_1150);
//            ftcLib_motorRR.setInverted(true);
            ftcLib_motorRR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//            ftcLib_motorRR.setRunMode(Motor.RunMode.PositionControl);



            motorLR = hwMap.get(DcMotorEx.class, "motorLR");
            motorLR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLR.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
            motorLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorLR.setPower(0);


            motorRF = hwMap.get(DcMotorEx.class, "motorRF");
            motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRF.setDirection(DcMotor.Direction.REVERSE);
            motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorRF.setPower(0);


            motorRR = hwMap.get(DcMotorEx.class, "motorRR");
            motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRR.setDirection(DcMotor.Direction.REVERSE);
            motorRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorRR.setPower(0);

            perpOdo = hwMap.get(DcMotor.class, "perp");
            perpOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            perpOdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            parOdo = hwMap.get(DcMotor.class, "par");
            parOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            parOdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        motorLift = hwMap.get(DcMotorEx.class, "motorLift");
        motorLift.setDirection(DcMotorEx.Direction.REVERSE);
        motorLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setTargetPosition(0);
        motorLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setPower(0);               // set motor power

        armMotor = hwMap.get(DcMotorEx.class, "armMotor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setPower(0);
        // set motor power

        climbMotorLeft = hwMap.get(DcMotorEx.class, "par");
        climbMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        climbMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        climbMotorLeft.setPower(0);
        climbMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        climbMotorRight = hwMap.get(DcMotorEx.class, "perp");
        climbMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        climbMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        climbMotorRight.setPower(0);
        climbMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoClimbLockLeft = hwMap.get(Servo.class, "climbLockLeft");
        servoClimbLockRight = hwMap.get(Servo.class, "climbLockRight");


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        servoClawLeft = hwMap.get(Servo.class, "servoClawLeft");
        servoClawRight = hwMap.get(Servo.class, "servoClawRight");
        servoClawWrist = hwMap.get(Servo.class, "servoClawWrist");
        servoIntakeActuator = hwMap.get(Servo.class, "servoIntakeActuator");
        servoIntake = hwMap.get(Servo.class, "servoIntake");
        servoDrone = hwMap.get(Servo.class, "servoArmActuatorB");

        clawSensorRight = hwMap.get(RevColorSensorV3.class, "clawSensorRight");
        clawSensorRight.initialize();
        clawSensorLeft = hwMap.get(RevColorSensorV3.class, "clawSensorLeft");
        clawSensorLeft.initialize();

        rightLedBackRed = hwMap.get(DigitalChannel.class, "rightLedBackRed");
        rightLedBackRed.setMode(DigitalChannel.Mode.OUTPUT);
        rightLedBackGreen = hwMap.get(DigitalChannel.class, "rightLedBackGreen");
        rightLedBackGreen.setMode(DigitalChannel.Mode.OUTPUT);

        rightLedFrontRed = hwMap.get(DigitalChannel.class, "rightLedFrontRed");
        rightLedFrontRed.setMode(DigitalChannel.Mode.OUTPUT);
        rightLedFrontGreen = hwMap.get(DigitalChannel.class, "rightLedFrontGreen");
        rightLedFrontGreen.setMode(DigitalChannel.Mode.OUTPUT);


        leftLedBackRed = hwMap.get(DigitalChannel.class, "leftLedBackRed");
        leftLedBackRed.setMode(DigitalChannel.Mode.OUTPUT);
        leftLedBackGreen = hwMap.get(DigitalChannel.class, "leftLedBackGreen");
        leftLedBackGreen.setMode(DigitalChannel.Mode.OUTPUT);


        leftLedFrontRed = hwMap.get(DigitalChannel.class, "leftLedFrontRed");
        leftLedFrontRed.setMode(DigitalChannel.Mode.OUTPUT);
        leftLedFrontGreen = hwMap.get(DigitalChannel.class, "leftLedFrontGreen");
        leftLedFrontGreen.setMode(DigitalChannel.Mode.OUTPUT);


        // init distance sensor
//        sensorLeft = hwMap.get(DistanceSensor.class, "sensorLeft");
//        sensorRight = hwMap.get(DistanceSensor.class, "sensorRight");


    }
}  // end of HWProfile Class