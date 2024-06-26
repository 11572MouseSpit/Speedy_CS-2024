package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.RevIMU;

// test comment from Christopher
@Config
public class Params {
    public static final double LIFT_POWER = 1;
    public static final double DRONE_ACTUATOR_POWER = .1;
    public static final double SENSOR_RIGHT_CLOSE_DISTANCE = 20;
    public static final double SENSOR_LEFT_CLOSE_DISTANCE = 15;
    public static final double CLAW_DEBOUNCE_TIME = 3;
    public static final double SLOW_TURN_SPEED = .3;
    public static final double SLOWEST_MOVE_SPEED = .175;
    public static final double SLOW_MOVE_SPEED = .3;
    public static final double FINGER_RELEASE_LEFT = .5;
    public static final double FINGER_HOLD_LEFT = 0;
    public static final double FINGER_RELEASE_RIGHT = 0;
    public static final double FINGER_HOLD_RIGHT = .5;
    public static final double CLAW_SCORE = .5;
    public static final double ARM_LEFT_SCORE = 0.55;
    public static final double ARM_RIGHT_SCORE = 0.47;
    public static final int ARM_UP_POS = 600;
    public static final int ARM_HALFWAY_POS = 200;
    public static final double ARM_EXTEND_OUT = .5;
    public static final double ARM_EXTEND_IN = 1;
    public static final double CLAW_DOWN_POS = 0;
    public static final double ARM_DOWN_POS = .90;
    public static final double ARM_ZERO_POS = 1;
    public static final double CLAW_WRIST_UP = .4;
    public static final double CLAW_WRIST_ZERO = 1;
    public static final double ARM_IDLE_POWER = .8;
    public static final int ARM_HIGH_SCORE_POS = 450;
    public static final double CLAW_WRIST_UP_HIGH_SCORE = .15;
    public static final double ARM_HALF_EXTEND = .65;
    public static final int ARM_MIDWAY_UP = 500;
    public static boolean armUp = false;
    public static Thread armResetThread = new Thread();
    public static final double CLAW_WRIST_DOWN = 1;
    public static int CLIMB_POS = 350;
    public static double EXTEND_STACK_5_RIGHT = 0.635;
    public static double EXTEND_STACK_5_LEFT = 0.365; //change to real value!
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
    public final int LIFT_MID_POSITION = 500;
    public final int LIFT_AUTO_SCORE = 75;
    public final int LIFT_HIGH_POSITION = 1600;
    public final int LIFT_MAX_HEIGHT = 4650;
    public final double LIFT_POWER_UP = 1;
    public final double LIFT_POWER_DOWN = 0.2;
    public final double LIFT_POSITION_TOLERANCE = 10;
    public final double LIFT_kP = 0.005;
    public final double LIFT_kI = 0.005;
    public final double LIFT_kD = 1.05;
    public final double LIFT_kF = 0.7;

    // Claw constants
    public final double CLAW_RIGHT_OPEN = 1;
    public final double CLAW_RIGHT_CLOSE = .4;
    public final double CLAW_RIGHT_OPEN_BOARD = .6;
    public final double CLAW_RIGHT_OPEN_BUCKET = .5;
    public final double CLAW_LEFT_OPEN = .4;
    public final double CLAW_LEFT_OPEN_BUCKET = .5;
    public final double CLAW_LEFT_OPEN_BOARD = 4;

    public final double CLAW_LEFT_CLOSE = 1;

    // Bucket constants
    public final double BUCKET_RESET = 0;
    public final double BUCKET_SCORE = 1;
    public final double BUCKET_AUTO_SCORE = 1;
    public final double BUCKET_LINE_UP = 0; // not used properly atm

    // Drone Constants
    public final double DRONE_LOAD = 1;
    public final double DRONE_FIRE = 0;

    //Arm constants
    public final double ARM_LEFT_EXTEND = .30;
    public final double ARM_LEFT_EXTEND_BLOCK = .45;
    public final double ARM_LEFT_EXTEND_LOW_IDLE = .35;
    public final double ARM_LEFT_IDLE = 0.65;
    public final double ARM_LEFT_RESET = 1;
    public final double ARM_RIGHT_EXTEND = 0.79;
    public final double ARM_RIGHT_EXTEND_BLOCK = .55;
    public final double ARM_RIGHT_EXTEND_LOW_IDLE = .65;
    public final double ARM_RIGHT_IDLE = 0.35;
    public final double ARM_RIGHT_RESET = .16;

    // wrist constants
    public final double WRIST_RESET = 0.5;
    public final double WRIST_LOAD_PIXELS = .48;
    public final double WRIST_EXTEND = 0.05;

    // Slide Constants
    public final double SLIDE_LEFT_EXTEND = 0.55;
    public final double SLIDE_LEFT_RESET = 1;
    public final double SLIDE_RIGHT_EXTEND = .45;
    public final double SLIDE_RIGHT_RESET = 0;

    // Finger constants
    public final double FINGER_OUT = .5;
    public final double FINGER_IN = 0;
    public boolean armDeployed = false;
    public boolean scoreHigh = false;


    /* Constructor */
    public Params(){

    }

    /* Initialize standard Hardware interfaces */
    public void init() {
    }
}  // end of HWProfile Class