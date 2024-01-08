package org.firstinspires.ftc.teamcode.Libs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.RRHWProfile;

@Config
public class RRMechOps {

    private RRHWProfile robot;
    public LinearOpMode opMode;

    /*
     * Constructor method
     */
    public RRMechOps(RRHWProfile myRobot, LinearOpMode myOpMode){
        robot = myRobot;
        opMode = myOpMode;

    }   // close RRMechOps constructor Method

    public void liftPos(int liftPos) {
        liftPos = Range.clip(liftPos, 0, 1290);

        robot.motorLift.setPower(1);
        robot.motorLift.setTargetPosition(liftPos);
    }

    public void resetBucket() {
        robot.servoBucketAxis.setPosition(robot.BUCKET_RESET);
    }

    public void closeClaw(){
        robot.servoGrabber.setPosition(robot.CLAW_CLOSE);
    }

    public void loadDrone() {
        robot.servoLauncher.setPosition(.1);
    }

    public void openClaw(){
        robot.servoGrabber.setPosition(robot.CLAW_OPEN);
    }

    public void liftPosition(int liftPosition, double power) {
        robot.motorLift.setTargetPosition(liftPosition);
//        robot.winchMotor.setPower(robot.WINCH_POWER);
    }

    public void resetLift(double power){
        robot.motorLift.setTargetPosition(robot.LIFT_RESET);
        robot.motorLift.setPower(power);
    }

}   // close the RRMechOps class