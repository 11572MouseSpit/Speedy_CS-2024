package org.firstinspires.ftc.teamcode.Libs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.Hardware.RRHWProfile;

@Config
public class RRMechOps {

    private RRHWProfile robot;
    public LinearOpMode opMode;
    public Params params;

    /*
     * Constructor method
     */
    public RRMechOps(RRHWProfile myRobot, LinearOpMode myOpMode, Params myParams){
        robot = myRobot;
        opMode = myOpMode;
        params = myParams;

    }   // close RRMechOps constructor Method

    public void liftPos(int liftPos) {
        liftPos = Range.clip(liftPos, 0, 1290);

        robot.motorLift.setPower(1);
        robot.motorLift.setTargetPosition(liftPos);
    }

    public void bucketReset() {
        armIdle();
        robot.servoBucket.setPosition(params.BUCKET_RESET);
    }

    public void bucketScore() {
        armIdle();
        robot.servoBucket.setPosition(params.BUCKET_SCORE);
    }

    public void loadPixels(){
        liftReset();
        bucketReset();
        armReset();
        wristPosition(params.WRIST_LOAD_PIXELS);
        opMode.sleep(100);
        clawRightOpen();
        clawLeftOpen();
        opMode.sleep(50);
        wristPosition(params.WRIST_EXTEND);
        armExtend();
    }

    public void wristPosition(double position) {
        robot.servoWrist.setPosition(position);
    }

    public void clawLeftOpen(){
        robot.servoClawLeft.setPosition(params.CLAW_LEFT_OPEN);
    }

    public void clawleftclose(){
        robot.servoClawLeft.setPosition(params.CLAW_LEFT_CLOSE);
    }

    public void clawRightOpen(){
        robot.servoClawLeft.setPosition(params.CLAW_RIGHT_OPEN);
    }

    public void clawRightClose(){
        robot.servoClawLeft.setPosition(params.CLAW_RIGHT_CLOSE);
    }

    public void armExtend(){
        robot.servoArmLeft.setPosition(params.ARM_LEFT_EXTEND);
        robot.servoArmRight.setPosition(params.ARM_RIGHT_EXTEND);
    }

    public void slidesExtend(){
        robot.servoSlideLeft.setPosition(params.SLIDE_LEFT_EXTEND);
        robot.servoSlideRight.setPosition(params.SLIDE_RIGHT_EXTEND);
    }

    public void slidesReset(){
        robot.servoSlideLeft.setPosition(params.SLIDE_LEFT_RESET);
        robot.servoSlideRight.setPosition(params.SLIDE_RIGHT_RESET);
    }

    public void armIdle(){
        robot.servoArmLeft.setPosition(params.ARM_LEFT_IDLE);
        robot.servoArmRight.setPosition(params.ARM_RIGHT_IDLE);
    }

    public void armReset(){
        robot.servoArmLeft.setPosition(params.ARM_LEFT_RESET);
        robot.servoArmRight.setPosition(params.ARM_RIGHT_RESET);
    }

    public void droneLoad() {
        robot.servoDrone.setPosition(params.DRONE_LOAD);
    }


    public void liftPosition(int liftPosition) {
        robot.motorLift.setPower(params.LIFT_POWER_DOWN);
        robot.motorLift.setTargetPosition(liftPosition);
    }

    public void liftReset(){
        armIdle();
        bucketReset();
        robot.motorLift.setPower(params.LIFT_POWER_DOWN);
        robot.motorLift.setTargetPosition(params.LIFT_RESET);
    }

}   // close the RRMechOps class