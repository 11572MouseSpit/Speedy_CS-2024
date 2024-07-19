package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.Hardware.RRHWProfile;
import org.firstinspires.ftc.teamcode.Libs.RRMechOps;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Motor Encoder Tester", group = "Competition")

public class MotorEncoderTester extends LinearOpMode {
    private final static RRHWProfile robot = new RRHWProfile();
    private final static Params params = new Params();
    private static int liftHoldPos = 0;
    private static boolean holdLift = false;
    private static final boolean debug = false;
    private static int liftPos = 0;

    @Override
    public void runOpMode(){
        double v1, v2, v3, v4, robotAngle;
        double theta;
        double theta2 = 180;
        int flippedHeadingAngle = 110;
        double r;
        double power=1;
        double rightX, rightY;
        double TargetRotation = 0;
        double OldRotation = 0;
        boolean canOpen = true;
        boolean rotateEnabled = false;
        boolean fieldCentric = true;
        FourBarPosition fourBarPosition = FourBarPosition.FOUR_BAR_IN;
        int targetPosition = 0;
        LinearOpMode opMode = this;
        boolean passthroughMode = false;
//        double liftPower = robot.LIFT_POWER_DOWN;
        ElapsedTime elapsedTime = new ElapsedTime();
        ElapsedTime elapsedTimeIn = new ElapsedTime();
        ElapsedTime elapsedTimeOut = new ElapsedTime();
        int clawAxisPos = 0;
        boolean leftClawOpen = false;
        boolean rightClawOpen = false;
        boolean rbCooldown = false;
        boolean dpadUpBtnCooldown = false;
        boolean dpadDownBtnCooldown = false;
        boolean yBtnCooldown = false;
        boolean bBtnCooldown = false;
        boolean rtBtnReleased = false;
        boolean lbCooldown = false;
        boolean aCooldown = false;
        boolean climbMode = false;
        ElapsedTime climbModeTime = new ElapsedTime();
        ElapsedTime clawLeftAutoWait = new ElapsedTime();
        ElapsedTime clawRightAutoWait = new ElapsedTime();
        int slowMode = 0;
        double RFrotatePower = params.TURN_SPEED;
        double slidesLeftExtend = params.SLIDE_LEFT_EXTEND;
        double slidesRightExtend = params.SLIDE_RIGHT_EXTEND;
        double LFrotatePower = -params.TURN_SPEED;
        double LRrotatePower = -params.TURN_SPEED;
        boolean firstRun = true;
        double RRrotatePower = params.TURN_SPEED;
        ElapsedTime sensorLeftDebounce = new ElapsedTime();
        ElapsedTime sensorRightDebounce = new ElapsedTime();
        int dronePos = 0;

        robot.init(hardwareMap, true);

        RRMechOps mechOps = new RRMechOps(robot, opMode, params);

        telemetry.addData("Ready to Run: ","GOOD LUCK");
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();

        params.scoreHigh = false;
        params.armDeployed = false;
//        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(!opModeIsActive()) {
            if(isStopRequested()) {
                break;
            }
        }

        while (opModeIsActive()) {

            /*******************************************
             ****** Mecanum Drive Control section ******
             *******************************************/

            if(firstRun) {
                clawRightAutoWait.startTime();
                robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.armMotor.setPower(0);

                firstRun = false;
            }


            telemetry.addData("Arm Encoder Pos: ", robot.armMotor.getCurrentPosition());
            telemetry.addData("motorLF Encoder Pos: ", robot.motorLF.getCurrentPosition());
            telemetry.addData("motorLR Encoder Pos: ", robot.motorLR.getCurrentPosition());
            telemetry.addData("motorRF Encoder Pos: ", robot.motorRF.getCurrentPosition());
            telemetry.addData("motorRR Encoder Pos: ", robot.motorRR.getCurrentPosition());
            telemetry.addData("Lift Encoder Pos: ", robot.motorLift.getCurrentPosition());
            telemetry.addData("Arm Current: ", robot.armMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}       // end of MSTeleop class