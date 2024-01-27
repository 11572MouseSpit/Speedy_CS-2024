package org.firstinspires.ftc.teamcode.Hardware;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// test comment from Christopher
public class Params {
    public static final double LIFT_POWER = 1;
    public static final double DRONE_ACTUATOR_POWER = .1;
    public static final double SENSOR_RIGHT_CLOSE_DISTANCE = 20;
    public static final double SENSOR_LEFT_CLOSE_DISTANCE = 15;
    public static final double CLAW_DEBOUNCE_TIME = 3;
    /* Public OpMode members. */
    public RevIMU imu = null;

    // Drive Constants
    public final double DRIVE_TICKS_PER_INCH = 44;      //temporary values => To be updated
    public final double TURN_SPEED = 0.5;
    public final double STRAFE_FACTOR = 1.1;
    public final double PAR_IN_PER_TICK = 0.00300257;
    public final double PERP_IN_PER_TICK = 0.00304183;


    // Lift Constants
    public final int LIFT_RESET = 0;
    public final int LIFT_LOW_POSITION = 380;
    public final int LIFT_MID_POSITION = 730;
    public final int LIFT_AUTO_SCORE = 300;
    public final int LIFT_HIGH_POSITION = 3500;
    public final int LIFT_MAX_HEIGHT = 6000;
    public final double LIFT_POWER_UP = 1;
    public final double LIFT_POWER_DOWN = 0.2;
    public final double LIFT_POSITION_TOLERANCE = 10;
    public final double LIFT_kP = 0.005;
    public final double LIFT_kI = 0.005;
    public final double LIFT_kD = 1.05;
    public final double LIFT_kF = 0.7;

    // Claw constants
    public final double CLAW_RIGHT_OPEN = .7;
    public final double CLAW_RIGHT_CLOSE = 0;
    public final double CLAW_LEFT_OPEN = .3;
    public final double CLAW_LEFT_CLOSE = 1;

    // Bucket constants
    public final double BUCKET_RESET = 1;
    public final double BUCKET_SCORE = 0;
    public final double BUCKET_AUTO_SCORE = 0;
    public final double BUCKET_LINE_UP = 1; // not used properly atm

    // Drone Constants
    public final double DRONE_LOAD = 1;
    public final double DRONE_FIRE = 0;

    //Arm constants
    public final double ARM_LEFT_EXTEND = .30;
    public final double ARM_LEFT_EXTEND_BLOCK = .45;
    public final double ARM_LEFT_EXTEND_LOW_IDLE = .35;
    public final double ARM_LEFT_IDLE = 0.65;
    public final double ARM_LEFT_RESET = 1;
    public final double ARM_RIGHT_EXTEND = 0.72;
    public final double ARM_RIGHT_EXTEND_BLOCK = .55;
    public final double ARM_RIGHT_EXTEND_LOW_IDLE = .65;
    public final double ARM_RIGHT_IDLE = 0.35;
    public final double ARM_RIGHT_RESET = 0;

    // wrist constants
    public final double WRIST_RESET = 0.5;
    public final double WRIST_LOAD_PIXELS = .25;
    public final double WRIST_EXTEND = 0.03;

    // Slide Constants
    public final double SLIDE_LEFT_EXTEND = 0.55;
    public final double SLIDE_LEFT_RESET = 1;
    public final double SLIDE_RIGHT_EXTEND = .45;
    public final double SLIDE_RIGHT_RESET = 0;
    public final double FINGER_OUT = 0;
    public final double FINGER_IN = 1;


    /* Constructor */
    public Params(){

    }

    /* Initialize standard Hardware interfaces */
    public void init() {
    }
}  // end of HWProfile Class