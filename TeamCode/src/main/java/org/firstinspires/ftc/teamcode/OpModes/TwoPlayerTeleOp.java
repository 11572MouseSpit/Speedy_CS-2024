package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.Hardware.RRHWProfile;
import org.firstinspires.ftc.teamcode.Libs.RRMechOps;

@TeleOp(name = "2 Player TeleOp", group = "Competition")

public class TwoPlayerTeleOp extends LinearOpMode {
    private final static RRHWProfile robot = new RRHWProfile();
    private final static Params params = new Params();

    @Override
    public void runOpMode(){
        double v1, v2, v3, v4, robotAngle;
        double theta;
        double theta2 = 180;
        double r;
        double power=1;
        double rightX, rightY;
        double TargetRotation = 0;
        double OldRotation = 0;
        int liftPos = 0;
        boolean canOpen = true;
        boolean rotateEnabled = false;
        boolean fieldCentric = true;
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
        boolean lbCooldown = false;
        double RFrotatePower = params.TURN_SPEED;
        double LFrotatePower = -params.TURN_SPEED;
        double LRrotatePower = -params.TURN_SPEED;
        double RRrotatePower = params.TURN_SPEED;


        robot.init(hardwareMap, true);

        RRMechOps mechOps = new RRMechOps(robot, opMode, params);

        telemetry.addData("Ready to Run: ","GOOD LUCK");
        telemetry.update();

        mechOps.slidesReset();
        mechOps.bucketReset();
        mechOps.armReset();
        mechOps.clawRightClose();
        mechOps.clawleftclose();

        waitForStart();

        while (opModeIsActive()) {

            /*******************************************
             ****** Mecanum Drive Control section ******
             *******************************************/
            if (fieldCentric) {             // verify that the user hasn't disabled field centric drive
                theta = params.imu.getAbsoluteHeading() + 90;
//                        robot.imu.getAngularOrientation().firstAngle + 90;
            } else {
                theta = 0;      // do not adjust for the angular position of the robot
            }

            robotAngle = Math.atan2(gamepad1.left_stick_y, (-gamepad1.left_stick_x)) - Math.PI / 4;
            rightX = gamepad1.right_stick_x;
            rightY = gamepad1.right_stick_y;
            r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);

            v1 = (r * Math.cos(robotAngle - Math.toRadians(theta + theta2)) - rightX + rightY);
            v2 = (r * Math.sin(robotAngle - Math.toRadians(theta + theta2)) + rightX + rightY);
            v3 = (r * Math.sin(robotAngle - Math.toRadians(theta + theta2)) - rightX + rightY);
            v4 = (r * Math.cos(robotAngle - Math.toRadians(theta + theta2)) + rightX + rightY);

            if(((params.imu.getAbsoluteHeading() - OldRotation >= TargetRotation && TargetRotation > 0) || (params.imu.getAbsoluteHeading() - OldRotation <= TargetRotation && TargetRotation < 0)) && rotateEnabled) {
                rotateEnabled = false;
                TargetRotation = 0;
                OldRotation = 0;
            }

            if(!rotateEnabled) {
                robot.motorLF.setPower(Range.clip((v1), -power, power));
                robot.motorRF.setPower(Range.clip((v2), -power, power));
                robot.motorLR.setPower(Range.clip((v3), -power, power));
                robot.motorRR.setPower(Range.clip((v4), -power, power));
            } else {
                robot.motorLF.setPower(LFrotatePower);
                robot.motorRF.setPower(RFrotatePower);
                robot.motorLR.setPower(LRrotatePower);
                robot.motorRR.setPower(RRrotatePower);
            }

            /*------------CLAW CONTROL------------*/
            if(gamepad2.left_bumper && lbCooldown == false) {
                lbCooldown = true;

                if(!leftClawOpen) {
                    mechOps.clawleftclose();
                    leftClawOpen = true;
                } else {
                    mechOps.clawleftclose();
                    leftClawOpen = false;
                }
            } else if(gamepad2.right_bumper && rbCooldown == false) {
                rbCooldown = true;

                if(!rightClawOpen) {
                    mechOps.clawRightOpen();
                    rightClawOpen = true;
                } else {
                    mechOps.clawRightClose();
                    rightClawOpen = false;
                }
            }

            if(gamepad2.right_stick_button) {
                mechOps.clawRightClose();
                mechOps.clawleftclose();
            }

            //cooldown management
            if(!gamepad2.left_bumper) {
                lbCooldown = false;
            }

            if(!gamepad2.right_bumper) {
                rbCooldown = false;
            }

            /*---------------4B CONTROL---------------*/

            if(gamepad2.x) {
                /*
                if(fourBarPosition == FourBarPosition.FOUR_BAR_IN || fourBarPosition == FourBarPosition.FOUR_BAR_MID) {
                    mechOps.clawRightClose();
                    mechOps.clawleftclose();
                    mechOps.armExtend();
                    sleep(20);
                    mechOps.slidesExtend();
                    elapsedTimeOut.reset();
                }
                */
            } else if(gamepad2.b) {
                mechOps.clawRightClose();
                mechOps.clawleftclose();
                mechOps.slidesReset();
                sleep(20);
                mechOps.armReset();
                elapsedTimeIn.reset();

//                passthroughMode = true;
            } else if (gamepad2.left_stick_button) {
                mechOps.clawleftclose();
                mechOps.clawRightClose();
                sleep(20);
                mechOps.armIdle();
            }

            if(elapsedTimeIn.time() > 1 && elapsedTimeIn.time() < 1.5 ) {
                if(!passthroughMode) {
                    mechOps.clawRightOpen();
                    mechOps.clawleftclose();
                }
            }

            if(elapsedTimeOut.time() > 1 && elapsedTimeOut.time() < 1.5 ) {
                if(!passthroughMode) {
                    mechOps.clawRightOpen();
                    mechOps.clawleftclose();
                }
            }

            /*---------------BUCKET CONTROL---------------*/

            if(gamepad2.y) {
                mechOps.bucketScore();
            } else if(gamepad2.a) {
                mechOps.bucketReset();
            }

            /*---------------LIFT CONTROL---------------*/

            if(gamepad2.dpad_right) {
                /**
                if(fourBarPosition == FourBarPosition.FOUR_BAR_IN) {
                    passthroughMode = true;
                    elapsedTime.reset();
                } else {
                    mechOps.liftPos(params.LIFT_MAX_POS);
                }
            } else if(gamepad2.dpad_left) {
                mechOps.bucketReset();
                if(fourBarPosition != FourBarPosition.FOUR_BAR_IN) mechOps.liftPos(params.LIFT_MIN_POS);
                 **/
            }

            /**
            if(gamepad2.right_trigger > .1 && fourBarPosition != FourBarPosition.FOUR_BAR_IN) {
                liftPos += 100;
            } else if(gamepad2.left_trigger > .1 && fourBarPosition != FourBarPosition.FOUR_BAR_IN) {
                liftPos -= 100;
            }
             **/

            if(passthroughMode) {
                if(elapsedTime.time() > .1 && elapsedTime.time() < .5) {
                    mechOps.clawRightOpen();
                    mechOps.clawleftclose();
                } else if(elapsedTime.time() > .5 && elapsedTime.time() < 1) {
                    mechOps.clawleftclose();
                    mechOps.clawRightClose();
                } else if(elapsedTime.time() > 1 && elapsedTime.time() < 2) {
                    mechOps.clawleftclose();
                    mechOps.clawRightClose();
                    mechOps.armIdle();

//                    fourBarPosition = FourBarPosition.FOUR_BAR_MID;
                } else if(elapsedTime.time() > 2) {
                    mechOps.liftPos(params.LIFT_HIGH_POSITION);
                    passthroughMode = false;
                }
            }

            // 90 degree turn

            // Provide user feedback
//            telemetry.addData("V1 = ", v1);
//            telemetry.addData("elapsed time = ", elapsedTime.time());
//            telemetry.addData("V2 = ", v2);
//            telemetry.addData("V3 = ", v3);
//            telemetry.addData("V4 = ", v4);
//            telemetry.addData("Motor Left Lift = ", robot.motorLeftLift.getCurrentPosition());
//            telemetry.addData("Motor Right Lift = ", robot.motorRightLift.getCurrentPosition());
//            telemetry.addData("Theta = ", theta);
//            telemetry.addData("Theta2 = ", theta);
//            telemetry.addData("IMU Value: ", theta);
//            telemetry.addData("robot rotation: ", OldRotation);
//            telemetry.update();

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}       // end of MSTeleop class