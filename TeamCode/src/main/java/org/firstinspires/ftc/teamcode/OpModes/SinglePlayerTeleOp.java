package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

<<<<<<< Updated upstream
=======
>>>>>>> Stashed changes

@TeleOp(name = "1 Player TeleOp", group = "Competition")

public class SinglePlayerTeleOp extends LinearOpMode {
    private final static HWProfile robot = new HWProfile();

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
        double RFrotatePower = robot.TURN_SPEED;
        double LFrotatePower = -robot.TURN_SPEED;
        double LRrotatePower = -robot.TURN_SPEED;
        double RRrotatePower = robot.TURN_SPEED;
        FourBarPosition fourBarPosition = FourBarPosition.FOUR_BAR_IN;


        robot.init(hardwareMap);

        DriveClass drive = new DriveClass(robot, opMode);

        telemetry.addData("Ready to Run: ","GOOD LUCK");
        telemetry.update();

        drive.retractFourBar();
        drive.resetBucket();
        drive.fourBarIn();
        drive.closeRightClaw();
        drive.closeLeftClaw();

        waitForStart();

        while (opModeIsActive()) {

            /*******************************************
             ****** Mecanum Drive Control section ******
             *******************************************/
            if (fieldCentric) {             // verify that the user hasn't disabled field centric drive
                theta = robot.imu.getAbsoluteHeading() + 90;
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

            if(((robot.imu.getAbsoluteHeading() - OldRotation >= TargetRotation && TargetRotation > 0) || (robot.imu.getAbsoluteHeading() - OldRotation <= TargetRotation && TargetRotation < 0)) && rotateEnabled) {
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
            if(gamepad1.left_bumper && lbCooldown == false) {
                lbCooldown = true;

                if(!leftClawOpen) {
                    drive.openLeftClaw();
                    leftClawOpen = true;
                } else {
                    drive.closeLeftClaw();
                    leftClawOpen = false;
                }
            } else if(gamepad1.right_bumper && rbCooldown == false) {
                rbCooldown = true;

                if(!rightClawOpen) {
                    drive.openRightClaw();
                    rightClawOpen = true;
                } else {
                    drive.closeRightClaw();
                    rightClawOpen = false;
                }
            }

            if(gamepad1.left_stick_button) {
                drive.closeRightClaw();
                drive.closeLeftClaw();
            }

            //cooldown management
            if(!gamepad1.left_bumper) {
                lbCooldown = false;
            }

            if(!gamepad1.right_bumper) {
                rbCooldown = false;
            }

            /*---------------4B CONTROL---------------*/

            if(gamepad1.x) {
                if(fourBarPosition == FourBarPosition.FOUR_BAR_IN || fourBarPosition == FourBarPosition.FOUR_BAR_MID) {
                    drive.closeRightClaw();
                    drive.closeLeftClaw();
                    drive.fourBarOut();
                    sleep(20);
                    drive.extendFourBar();
                    fourBarPosition = FourBarPosition.FOUR_BAR_OUT;
                    elapsedTimeOut.reset();
                }
            } else if(gamepad1.b) {
                drive.closeRightClaw();
                drive.closeLeftClaw();
                drive.retractFourBar();
                sleep(20);
                drive.fourBarIn();
                fourBarPosition = FourBarPosition.FOUR_BAR_IN;
                elapsedTimeIn.reset();

//                passthroughMode = true;
            } else if (gamepad1.right_stick_button) {
                drive.closeLeftClaw();
                drive.closeRightClaw();
                sleep(20);
                drive.fourBarMid();
                fourBarPosition = FourBarPosition.FOUR_BAR_MID;
            }

            if(elapsedTimeIn.time() > 1 && elapsedTimeIn.time() < 1.5 ) {
                if(fourBarPosition == FourBarPosition.FOUR_BAR_IN && !passthroughMode) {
                    drive.openRightClaw();
                    drive.openLeftClaw();
                }
            }

            if(elapsedTimeOut.time() > 1 && elapsedTimeOut.time() < 1.5 ) {
                if(fourBarPosition == FourBarPosition.FOUR_BAR_OUT && !passthroughMode) {
                    drive.openRightClaw();
                    drive.openLeftClaw();
                }
            }

            /*---------------BUCKET CONTROL---------------*/

            if(gamepad1.y) {
                drive.bucketScore();
            } else if(gamepad1.a) {
                drive.resetBucket();
            }

            /*---------------LIFT CONTROL---------------*/

            if(gamepad1.dpad_right) {
                if(fourBarPosition == FourBarPosition.FOUR_BAR_IN) {
                    passthroughMode = true;
                    elapsedTime.reset();
                } else {
                    drive.liftPos(robot.LIFT_MAX_POS);
                }
            } else if(gamepad1.dpad_left) {
                drive.resetBucket();
                if(fourBarPosition != FourBarPosition.FOUR_BAR_IN) drive.liftPos(robot.LIFT_MIN_POS);
            }

            if(gamepad1.right_trigger > .1 && fourBarPosition != FourBarPosition.FOUR_BAR_IN) {
                liftPos += 20;
            } else if(gamepad1.left_trigger > .1 && fourBarPosition != FourBarPosition.FOUR_BAR_IN) {
                liftPos -= 20;
            }

            if(passthroughMode) {
                if(elapsedTime.time() > .1 && elapsedTime.time() < .5) {
                    drive.openRightClaw();
                    drive.openLeftClaw();
                } else if(elapsedTime.time() > .5 && elapsedTime.time() < 1) {
                    drive.closeLeftClaw();
                    drive.closeRightClaw();
                } else if(elapsedTime.time() > 1 && elapsedTime.time() < 2) {
                    drive.closeLeftClaw();
                    drive.closeRightClaw();
                    drive.fourBarMid();

                    fourBarPosition = FourBarPosition.FOUR_BAR_MID;
                } else if(elapsedTime.time() > 2) {
                    drive.liftPos(robot.LIFT_MAX_POS);
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
            telemetry.addData("fourbar not IN", fourBarPosition != FourBarPosition.FOUR_BAR_IN);
            telemetry.update();

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}       // end of MSTeleop class