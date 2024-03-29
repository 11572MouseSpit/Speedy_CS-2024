package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.Hardware.RRHWProfile;
import org.firstinspires.ftc.teamcode.Libs.RRMechOps;

@TeleOp(name = "1 Player TeleOp", group = "Competition")

public class SinglePlayerTeleOp extends LinearOpMode {
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
        boolean lbCooldown = false;
        boolean leftReleased = false;
        boolean rightReleased = false;
        boolean slowMode = false;
        double RFrotatePower = params.TURN_SPEED;
        double LFrotatePower = -params.TURN_SPEED;
        double LRrotatePower = -params.TURN_SPEED;
        double RRrotatePower = params.TURN_SPEED;
        ElapsedTime sensorLeftDebounce = new ElapsedTime();
        ElapsedTime sensorRightDebounce = new ElapsedTime();

        robot.init(hardwareMap, true);

        RRMechOps mechOps =new RRMechOps(robot, opMode, params);

        telemetry.addData("Ready to Run: ","GOOD LUCK");
        telemetry.update();

        mechOps.armReset();
        mechOps.bucketReset();
        mechOps.armReset();
        mechOps.clawRightClose();
        mechOps.clawleftclose();

        mechOps.armReset();
        mechOps.armReset();

        waitForStart();

        while (opModeIsActive()) {

            /*******************************************
             ****** Mecanum Drive Control section ******
             *******************************************/
            if (fieldCentric) {             // verify that the user hasn't disabled field centric drive
                theta = robot.imu.getAbsoluteHeading() + 180 ;
//                        robot.imu.getAngularOrientation().firstAngle + 90;
            } else {
                theta = 0;      // do not adjust for the angular position of the robot
            }

            robotAngle = Math.atan2(gamepad1.left_stick_y, (-gamepad1.left_stick_x)) - Math.PI / 4;
            rightX = gamepad1.right_stick_x;
            rightY = gamepad1.right_stick_y;
            r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);

            if(slowMode == true) {
                rightX *= params.SLOW_TURN_SPEED;
                rightY *= params.SLOW_TURN_SPEED;
                r *= params.SLOW_MOVE_SPEED;
            }

            v1 = (r * Math.cos(robotAngle - Math.toRadians(theta + theta2)) - rightX + rightY);
            v2 = (r * Math.sin(robotAngle - Math.toRadians(theta + theta2)) + rightX + rightY);
            v3 = (r * Math.sin(robotAngle - Math.toRadians(theta + theta2)) - rightX + rightY);
            v4 = (r * Math.cos(robotAngle - Math.toRadians(theta + theta2)) + rightX + rightY);

            if(((robot.imu.getAbsoluteHeading() - OldRotation >= TargetRotation && TargetRotation > 0) || (robot.imu.getAbsoluteHeading() - OldRotation <= TargetRotation && TargetRotation < 0)) && rotateEnabled) {
                rotateEnabled = false;
                TargetRotation = 0;
                OldRotation = 0;
            }

            if(!rotateEnabled)  {
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
            if(mechOps.getColor(robot.clawSensorRight) && fourBarPosition == FourBarPosition.FOUR_BAR_OUT) {
                if(sensorRightDebounce.time() >= params.CLAW_DEBOUNCE_TIME) {
                    if(rightClawOpen) {
                        gamepad1.rumble(500);
                    }
                    mechOps.clawRightClose();
                    sensorRightDebounce.reset();
                    rightClawOpen = false;
                }
            }

            if(mechOps.getColor(robot.clawSensorLeft) && fourBarPosition == FourBarPosition.FOUR_BAR_OUT) {
                if(sensorLeftDebounce.time() >= params.CLAW_DEBOUNCE_TIME) {
                    if(leftClawOpen) {
                        gamepad1.rumble(500);
                    }
                    mechOps.clawleftclose();
                    sensorLeftDebounce.reset();
                    leftClawOpen = false;
                }
            }

            if(gamepad1.left_bumper && lbCooldown == false) {
                lbCooldown = true;

                if(mechOps.bucketScored) {
                    mechOps.fingerReleaseRight();
                    leftReleased = true;
                }

                if(!leftClawOpen) {
                    if(fourBarPosition == FourBarPosition.FOUR_BAR_IN) {
                    mechOps.clawleftopenBucket();
                    } else {
                        mechOps.clawLeftOpen();
                    }
                    leftClawOpen = true;
                } else {
                    mechOps.clawleftclose();
                    leftClawOpen = false;
                }
            } else if(gamepad1.right_bumper && rbCooldown == false) {
                rbCooldown = true;

                if(mechOps.bucketScored) {
                    mechOps.fingerReleaseLeft();
                    rightReleased = true;
                }

                if(!rightClawOpen) {
                    if(fourBarPosition == FourBarPosition.FOUR_BAR_IN) {
                        mechOps.clawRightOpenBucket();
                    } else {
                        mechOps.clawRightOpen();
                    }
                    rightClawOpen = true;
                } else {
                    mechOps.clawRightClose();
                    rightClawOpen = false;
                }
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
                    mechOps.clawRightClose();
                    mechOps.clawleftclose();
                    rightClawOpen = false;
                    leftClawOpen = false;
                    sleep(20);
                    mechOps.armExtend();
                    mechOps.slidesExtend();
                    mechOps.wristPosition(params.WRIST_EXTEND);
                    fourBarPosition = FourBarPosition.FOUR_BAR_OUT;
                    elapsedTimeOut.reset();
                }


            } else if(gamepad1.b) {
                mechOps.clawRightClose();
                mechOps.clawleftclose();
                rightClawOpen = false;
                leftClawOpen = false;
                mechOps.slidesReset();
                mechOps.wristPosition(params.WRIST_LOAD_PIXELS);
                sleep(20);
                fourBarPosition = FourBarPosition.FOUR_BAR_IN;
                mechOps.armReset();
                elapsedTimeIn.reset();
//                passthroughMode = true;
            } else if (gamepad1.right_stick_button || gamepad1.left_stick_button) {
                mechOps.clawleftclose();
                mechOps.clawRightClose();
                rightClawOpen = false;
                leftClawOpen = false;
            }

//            if(elapsedTimeIn.time() > 1 && elapsedTimeIn.time() < 1.5 ) {
//                if(!passthroughMode && fourBarPosition == FourBarPosition.FOUR_BAR_IN) {
//                    mechOps.clawRightOpen();
//                    mechOps.clawLeftOpen();
//                } else {
//                    mechOps.clawleftclose();
//                    mechOps.clawRightClose();
//                }
//            }

            if(elapsedTimeOut.time() > 1 && elapsedTimeOut.time() < 1.5 ) {
                if(!passthroughMode && fourBarPosition == FourBarPosition.FOUR_BAR_OUT) {
                    mechOps.clawRightOpen();
                    mechOps.clawLeftOpen();
                    rightClawOpen = true;
                    leftClawOpen = true;
                } else {
                    mechOps.clawRightClose();
                    mechOps.clawleftclose();
                    rightClawOpen = false;
                    leftClawOpen = false;
                }
            }

            /*---------------BUCKET CONTROL---------------*/

            if(gamepad1.y) {
                mechOps.bucketScore();
            } else if(gamepad1.a) {
                mechOps.bucketReset();
            }

            if(leftReleased && rightReleased) {
                new Thread(() -> {
                    try {
                        Thread.sleep(250);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    mechOps.bucketReset();
                }).start();
                leftReleased = false;
                rightReleased = false;
            }

            /*---------------LIFT CONTROL---------------*/

            if(gamepad1.dpad_right) {

                if(fourBarPosition == FourBarPosition.FOUR_BAR_IN) {
                    passthroughMode = true;
                    elapsedTime.reset();
                } else {
                    liftPos = params.LIFT_HIGH_POSITION;
                    mechOps.liftPosition(params.LIFT_HIGH_POSITION);
                }

            } else if(gamepad1.dpad_left) {
                mechOps.bucketReset();
                mechOps.armIdle();
                fourBarPosition = FourBarPosition.FOUR_BAR_MID;
                if(fourBarPosition != FourBarPosition.FOUR_BAR_IN) {
                    liftPos = 0;
                    mechOps.liftPosition(0);
                }
            }

//            if(gamepad1.right_bumper) {
//                mechOps.liftPosition(params.LIFT_MAX_HEIGHT);
//                mechOps.armIdle();
//                sleep(20);
//                mechOps.bucketScore();
//            } else if(gamepad1.left_bumper) {
//                mechOps.liftPosition(params.LIFT_MID_POSITION);
//            }

            if(gamepad1.left_trigger > .1 && gamepad1.right_trigger > .1) {
                slowMode = true;
                liftPos = 0;
                mechOps.liftPosition(0);
            } else if(gamepad1.right_trigger > .1 && fourBarPosition != FourBarPosition.FOUR_BAR_IN) {
                liftPos += 100;
                mechOps.liftPosition(liftPos);
            } else if(gamepad1.left_trigger > .1) {
                liftPos -= 100;
                mechOps.liftPosition(liftPos);
            } else {
                slowMode = false;
            }

            if(!(gamepad1.left_trigger > .1) && !(gamepad1.right_trigger > .1)) {
//                liftPos = robot.motorLift.getCurrentPosition();
//                mechOps.liftPosition(liftPos);
            }

            if(passthroughMode) {
                if(elapsedTime.time() > .1 && elapsedTime.time() < .5) {
                    if(!leftClawOpen && !rightClawOpen) {
                        mechOps.clawRightOpenBucket();
                        mechOps.clawleftopenBucket();
                        rightClawOpen = true;
                        leftClawOpen = true;
                    }
                } else if(elapsedTime.time() > .5 && elapsedTime.time() < 1) {
                    mechOps.clawleftclose();
                    mechOps.clawRightClose();
                    rightClawOpen = false;
                    leftClawOpen = false;
                } else if(elapsedTime.time() > 1 && elapsedTime.time() < 2) {
                    mechOps.clawleftclose();
                    mechOps.clawRightClose();
                    rightClawOpen = false;
                    leftClawOpen = false;
                    mechOps.armIdle();

                    fourBarPosition = FourBarPosition.FOUR_BAR_MID;
                } else if(elapsedTime.time() > 2) {
                    liftPos = params.LIFT_MID_POSITION;
                    mechOps.liftPosition(params.LIFT_MID_POSITION);
                    mechOps.fingerHoldLeft();
                    mechOps.fingerHoldRight();
                    leftReleased = false;
                    rightReleased = false;
                    mechOps.bucketScore();
                    passthroughMode = false;
                }
            }

            //--------------------------FINGER CONTROL--------------------------//

            if(gamepad1.dpad_up) {
                mechOps.fingerOut();
            } else if(gamepad1.dpad_down) {
                mechOps.fingerIn();
            }

            mechOps.leftLEDState(!leftClawOpen);
            mechOps.rightLEDState(!rightClawOpen);

            // 90 degree turn

            // Provide user feedback
            telemetry.addData("Lift pos = ", liftPos);
//            telemetry.addData("lift motor 1 position: ", robot.liftMotorGroup.getPositions().get(1));
//            telemetry.addData("lift motor 1 target position: ", robot.liftMotorGroup.getPositions().get(1));
//            telemetry.addData("lift motor 0 position: ", robot.liftMotorGroup.getPositions().get(0));
//            telemetry.addData("lift motor 0 speed: ", robot.liftMotorGroup.getSpeeds().get(0));
            telemetry.addData("first lift pos = ", robot.motorLift.getCurrentPosition());
            telemetry.addData("first lift current = ", robot.motorLift.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("second lift pos = ", robot.secondMotorLift.getCurrentPosition());
            telemetry.addData("second lift current = ", robot.secondMotorLift.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("left colors: ", Math.round(robot.clawSensorLeft.getNormalizedColors().toColor() / 100));
//            telemetry.addData("elapsed time = ", elapsedTime.time());
//            telemetry.addData("V2 = ", v2);
//            telemetry.addData("V3 = ", v3);
//            telemetry.addData("V4 = ", v4);
//            telemetry.addData("Motor Left Lift = ", robot.motorLeftLift.getCurrentPosition());
//            telemetry.addData("Motor Right Lift = ", robot.motorRightLift.getCurrentPosition());
//            telemetry.addData("Theta = ", theta);
//            telemetry.addData("Theta2 = ", theta);
//            telemetry.addData("IMU Value: ", theta);
//            telemetry.addData("fourbar not IN", fourBarPosition != FourBarPosition.FOUR_BAR_IN);
            liftPos = Range.clip(liftPos, 0, params.LIFT_MAX_HEIGHT);
            telemetry.update();

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}       // end of MSTeleop class