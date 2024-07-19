package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.RevIMU;

// test comment from Christopher
@Config
public class Params {
    public static final int ARM_AUTO_MID_POS = -1300;
    public static double SLOW_MODE_SPEED = .5;
    public static double NORMAL_SPEED = 1;
    public static int DRONE_FIRE_ARM_POS = -525;
    public static long CLAW_AUTO_GRAB_COOLDOWN = 1750;
    public static int ARM_SLIGHTLY_OFF_GROUND = -350;
    public static int ARM_FLOAT_POS = -600;
    public static double ARM_LOW_POWER = .6;
    public static int ARM_RESET_POS = -900;
    public static int ARM_ABOVE_BLOCK = -650;
    public static int ARM_STACK_5_POS = -300;
    public static int ARM_STACK_4_POS = -265;
    public static double INTAKE_DEPLOY_POS = 0;
    public static double IDLE_CLIMB_DOWN = -.1;
    public static double INTAKE_DEPLOY_STACK_POS = 1 - .3;

    public static final double INTAKE_ZERO_POS = 0;
    public static final double INTAKE_RETRACT_POS = .2;
    public static final double INTAKE_RESET_POS = .45;
    public static final double INTAKE_SPEED = 0;
    public static final double INTAKE_STOP = .5;
    public static final double CLAW_SENSE_DISTANCE = 1.5;
    public static final double CLIMB_HOME_AMPERAGE = 0; //was 1.25
    public static final int LIFT_STACK_5 = 400;
    public static double CLAW_FINGER_POS = .6;
    public static double LIFT_POWER = 1;
    public static double DRONE_ACTUATOR_POWER = .1;
    public static double SENSOR_RIGHT_CLOSE_DISTANCE = 20;
    public static double SENSOR_LEFT_CLOSE_DISTANCE = 15;
    public static double CLAW_DEBOUNCE_TIME = 3;
    public static double SLOW_TURN_SPEED = .3;
    public static double SLOWEST_MOVE_SPEED = .175;
    public static double SLOW_MOVE_SPEED = .3;
    public static double FINGER_RELEASE_LEFT = .5;
    public static double FINGER_HOLD_LEFT = 0;
    public static double FINGER_RELEASE_RIGHT = 0;
    public static double FINGER_HOLD_RIGHT = .5;
    public static double CLAW_SCORE = .5;
    public static double ARM_LEFT_SCORE = 0.55;
    public static double ARM_RIGHT_SCORE = 0.47;
    public static int ARM_UP_POS = -1850;
    public static int ARM_UP_AUTO_POS = -1750;
    public static int ARM_HALFWAY_POS = -900;
    public static double ARM_EXTEND_OUT = .5;
    public static double ARM_EXTEND_IN = 1;
    public static double CLAW_DOWN_POS = 0;
    public static int ARM_DOWN_POS = -350;
    public static int ARM_DOWN_AUTO_POS = 0;
    public static double ARM_ZERO_POS = 1;
    public static double CLAW_WRIST_UP = .4;
    public static double CLAW_WRIST_MID_POS = .7;
    public static double CLAW_WRIST_UP_AUTO = .4;
    public static double CLAW_WRIST_ZERO = .95;
    public static double ARM_IDLE_POWER = .8;
    public static int ARM_HIGH_SCORE_POS = -1500;
    public static double CLAW_WRIST_UP_HIGH_SCORE = .4;
    public static double ARM_HALF_EXTEND = .65;
    public static int ARM_MIDWAY_UP = -500;
    public static double PID_TUNE_MOTOR_POWER = .5;
    public static boolean armUp = false;
    public static Thread armResetThread = new Thread();
    public static double CLAW_WRIST_DOWN = 1;
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
    public final int LIFT_LOW_2_POSITION = 650;
    public final int LIFT_LOW_1_POSITION = 450;
    public static int LIFT_MID_POSITION = 2000;
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
    public final double CLAW_RIGHT_OPEN = .25;
    public final double CLAW_RIGHT_OPEN_AUTO = .7;
    public final double CLAW_RIGHT_CLOSE = 0;
    public final double CLAW_RIGHT_OPEN_BOARD = .4;
    public final double CLAW_RIGHT_OPEN_BUCKET = .5;
    public final double CLAW_LEFT_OPEN = .8;
    public final double CLAW_LEFT_OPEN_AUTO = .3 ;
    public final double CLAW_LEFT_OPEN_BUCKET = .5;
    public final double CLAW_LEFT_OPEN_BOARD = .6;

    public final double CLAW_LEFT_CLOSE = 1;

    // Bucket constants
    public final double BUCKET_RESET = 0;
    public final double BUCKET_SCORE = 1;
    public final double BUCKET_AUTO_SCORE = 1;
    public final double BUCKET_LINE_UP = 0; // not used properly atm

    // Drone Constants
    public final double DRONE_LOAD = 1;
    public final double DRONE_FIRE = 0;
    public static double LOCK_SERVO_POS = 1;
    public static double UNLOCK_SERVO_POS = .6;

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
    public boolean pickupStack = false;
    public boolean scoreHigh = false;


    /* Constructor */
    public Params(){

    }

    /* Initialize standard Hardware interfaces */
    public void init() {
    }
}  // end of HWProfile Class