package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// test comment from Christopher
public class RRHWProfile {
    /* Public OpMode members. */

    public Servo servoGrabber = null;
    public Servo clawAxis = null;

    public DistanceSensor sensorLeft;
    public DistanceSensor sensorRight;

    public Servo droneLauncher = null;
    public Servo servoLauncher = null;
    public Servo servoBucketAxis = null;

    public DcMotorEx motorLift = null;
    //    public final DistanceSensor armSensor = null;


    public final double DRIVE_TICKS_PER_INCH = 44;      //temporary values => To be updated
    public final int LIFT_RESET = 0;
    public final int LIFT_MAX_HEIGHT = 1300;
    public final double LIFT_POWER_UP = 1;
    public final double LIFT_POWER_DOWN = 1;
    public final double LIFT_POSITION_TOLERANCE = 10;
    public final double LIFT_kP = 0.005;
    public final double LIFT_kI = 0.005;
    public final double LIFT_kD = 1.05;
    public final double CONE_DISTANCE = 5;
    public final double LIFT_kF = 0.7;
    public final double WAIT_DRIVE_TO_CONE = 1;
    public final double CLAW_OPEN = .22;
//    public final double CLAW_OPEN = .35;
    public final double CLAW_CLOSE = 0;
    public final double BUCKET_RESET = 0;
    public final double BUCKET_SCORE = 0.75;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;

    /* Constructor */
    public RRHWProfile(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        motorLift = hwMap.get(DcMotorEx.class, "motorSlides");
        motorLift.setDirection(DcMotorEx.Direction.REVERSE);
        motorLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setTargetPosition(0);
        motorLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setPower(0);               // set motor power

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        servoGrabber = hwMap.get(Servo.class, "servoGrabber");
        servoLauncher = hwMap.get(Servo.class, "launcherServo");
        droneLauncher = hwMap.get(Servo.class, "droneLauncher");
        servoBucketAxis = hwMap.get(Servo.class, "servoBucketAxis");
        clawAxis = hwMap.get(Servo.class, "clawAxisServo");

        // init distance sensor
        sensorLeft = hwMap.get(DistanceSensor.class, "sensorLeft");
        sensorRight = hwMap.get(DistanceSensor.class, "sensorRight");

    }
}  // end of HWProfile Class