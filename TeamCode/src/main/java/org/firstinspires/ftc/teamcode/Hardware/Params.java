package org.firstinspires.ftc.teamcode.Hardware;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// test comment from Christopher
public class Params {
    /* Public OpMode members. */
    public RevIMU imu = null;

    // Drive Constants
    public final double DRIVE_TICKS_PER_INCH = 44;      //temporary values => To be updated
    public final double TURN_SPEED = 0.5;
    public final double STRAFE_FACTOR = 1.1;

    // Lift Constants
    public final int LIFT_RESET = 0;
    public final int LIFT_LOW_POSITION = 380;
    public final int LIFT_MID_POSITION = 730;
    public final int LIFT_HIGH_POSITION = 1270;
    public final int LIFT_MAX_HEIGHT = 1300;
    public final double LIFT_POWER_UP = 1;
    public final double LIFT_POWER_DOWN = 0.2;
    public final double LIFT_POSITION_TOLERANCE = 10;
    public final double LIFT_kP = 0.005;
    public final double LIFT_kI = 0.005;
    public final double LIFT_kD = 1.05;
    public final double LIFT_kF = 0.7;

    // Claw constants
    public final double CLAW_LEFT_OPEN = 0;
    public final double CLAW_LEFT_CLOSE = 0.7;
    public final double CLAW_RIGHT_OPEN = 0.8;
    public final double CLAW_RIGHT_CLOSE = 0.1;

    // Bucket constants
    public final double BUCKET_RESET = 1;
    public final double BUCKET_SCORE = 0.4;

    // Drone Constants
    public final double DRONE_LOAD = 0.1;
    public final double DRONE_FIRE = 0.5;

    //Arm constants
    public final double ARM_LEFT_EXTEND = 0.5;
    public final double ARM_LEFT_IDLE = 0.5;
    public final double ARM_LEFT_RESET = 0.5;
    public final double ARM_RIGHT_EXTEND = 0.5;
    public final double ARM_RIGHT_IDLE = 0.5;
    public final double ARM_RIGHT_RESET = 0.5;

    // wrist constants
    public final double WRIST_RESET = 0.5;
    public final double WRIST_LOAD_PIXELS = 0.5;
    public final double WRIST_EXTEND = 0.5;

    // Slide Constants
    public final double SLIDE_LEFT_EXTEND = 0.55;
    public final double SLIDE_LEFT_RESET = 1;
    public final double SLIDE_RIGHT_EXTEND = 0.5;
    public final double SLIDE_RIGHT_RESET = 0;


    /* Constructor */
    public Params(){

    }

    /* Initialize standard Hardware interfaces */
    public void init() {
    }
}  // end of HWProfile Class