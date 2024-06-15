package org.firstinspires.ftc.teamcode.Hardware;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public Servo servoDrone = null;
    public Servo servoArmActuatorA = null;
    public Servo servoArmActuatorB = null;
    public Servo servoClawWrist = null;
    public Servo servoArmExtend = null;



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

        if(driveMotors) {
            imu = new RevIMU(hwMap);
            imu.init();

            // Define and Initialize Motors
            motorLF = hwMap.get(DcMotorEx.class, "motorLF");
            motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLF.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorLF.setPower(0);


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

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        servoClawLeft = hwMap.get(Servo.class, "servoClawLeft");
        servoClawRight = hwMap.get(Servo.class, "servoClawRight");
        servoArmActuatorA = hwMap.get(Servo.class, "servoArmActuatorA");
        servoArmActuatorB = hwMap.get(Servo.class, "servoArmActuatorB");
        servoClawWrist = hwMap.get(Servo.class, "servoClawWrist");
        servoArmExtend = hwMap.get(Servo.class, "servoArmSlides");

        clawSensorRight = hwMap.get(RevColorSensorV3.class, "clawSensorRight");
        clawSensorLeft = hwMap.get(RevColorSensorV3.class, "clawSensorLeft");

        // init distance sensor
//        sensorLeft = hwMap.get(DistanceSensor.class, "sensorLeft");
//        sensorRight = hwMap.get(DistanceSensor.class, "sensorRight");


    }
}  // end of HWProfile Class