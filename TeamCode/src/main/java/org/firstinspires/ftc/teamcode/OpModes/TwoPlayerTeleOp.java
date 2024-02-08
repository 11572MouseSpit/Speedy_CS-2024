package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
        boolean climbMode = false;
        ElapsedTime climbModeTime = new ElapsedTime();
        int slowMode = 0;
        double RFrotatePower = params.TURN_SPEED;
        double slidesLeftExtend = params.SLIDE_LEFT_EXTEND;
        double slidesRightExtend = params.SLIDE_RIGHT_EXTEND;
        double LFrotatePower = -params.TURN_SPEED;
        double LRrotatePower = -params.TURN_SPEED;
        double RRrotatePower = params.TURN_SPEED;
        ElapsedTime sensorLeftDebounce = new ElapsedTime();
        ElapsedTime sensorRightDebounce = new ElapsedTime();
        int dronePos = 0;

        robot.init(hardwareMap, true);

        RRMechOps mechOps =new RRMechOps(robot, opMode, params);

        telemetry.addData("Ready to Run: ","GOOD LUCK");
        telemetry.update();

        mechOps.armReset();
        mechOps.bucketReset();
        mechOps.armReset();
        mechOps.slidesReset();
        mechOps.clawRightClose();
        mechOps.clawleftclose();

        mechOps.armReset();
        mechOps.armReset();

        while(!opModeIsActive()) {
            mechOps.armReset();
            if(isStopRequested()) {
                break; 
            }
        }

        while (opModeIsActive()) {

            /*******************************************
             ****** Mecanum Drive Control section ******
             *******************************************/
            if (fieldCentric) {             // verify that the user hasn't disabled field centric drive
                theta = robot.imu.getAbsoluteHeading() + 180;
//                        robot.imu.getAngularOrientation().firstAngle + 90;
            } else {
                theta = 0;      // do not adjust for the angular position of the robot
            }

            robotAngle = Math.atan2(gamepad1.left_stick_y, (-gamepad1.left_stick_x)) - Math.PI / 4;
            rightX = gamepad1.right_stick_x;
            rightY = gamepad1.right_stick_y;


            r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            if(slowMode == 1) {
                rightX *= params.SLOW_TURN_SPEED;
                rightY *= params.SLOW_TURN_SPEED;
                r *= params.SLOW_MOVE_SPEED;
            } else if(slowMode == 2) {
                rightX *= params.SLOW_TURN_SPEED;
                rightY *= params.SLOW_TURN_SPEED;
                r *= params.SLOWEST_MOVE_SPEED;
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
                        gamepad2.rumble(500);
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
                        gamepad2.rumble(500);
                        gamepad1.rumble(500);
                    }
                    mechOps.clawleftclose();
                    sensorLeftDebounce.reset();
                    leftClawOpen = false;
                }
            }
            if(gamepad1.left_bumper) {
                if(mechOps.bucketScored) {
                    mechOps.fingerReleaseRight();
                }
            } else if(gamepad1.right_bumper) {
                if(mechOps.bucketScored) {
                    mechOps.fingerReleaseLeft();
                }
            }


            if(gamepad2.left_bumper && lbCooldown == false) {
                lbCooldown = true;

                    if (!leftClawOpen) {
                        if (fourBarPosition == FourBarPosition.FOUR_BAR_IN) {
                            mechOps.clawleftopenBucket();
                        } else {
                            mechOps.clawLeftOpen();
                        }
                        leftClawOpen = true;
                    } else {
                        mechOps.clawleftclose();
                        leftClawOpen = false;
                    }
            } else if(gamepad2.right_bumper && rbCooldown == false) {
                rbCooldown = true;

                    if (!rightClawOpen) {
                        if (fourBarPosition == FourBarPosition.FOUR_BAR_IN) {
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
            if(!gamepad2.left_bumper) {
                lbCooldown = false;
            }

            if(!gamepad2.right_bumper) {
                rbCooldown = false;
            }
            /*---------------SLOW MODE---------------*/

            if(gamepad1.right_trigger > .1) {
                slowMode = 1;
            } else if(gamepad1.left_trigger > .1) {
                slowMode = 2;
            } else {
                slowMode = 0;
            }

            /*---------------4B CONTROL---------------*/

            if(gamepad2.right_stick_y < -.1 && fourBarPosition == FourBarPosition.FOUR_BAR_OUT) {
                slidesLeftExtend -= .1;
                slidesRightExtend += .1;

                slidesLeftExtend = Range.clip(slidesLeftExtend, params.SLIDE_LEFT_EXTEND, params.SLIDE_LEFT_RESET);
                slidesRightExtend = Range.clip(slidesRightExtend, params.SLIDE_RIGHT_RESET, params.SLIDE_RIGHT_EXTEND);

                mechOps.slidesCustomExtension(slidesLeftExtend, slidesRightExtend);
            } else if(gamepad2.right_stick_y > .1 && fourBarPosition == FourBarPosition.FOUR_BAR_OUT) {
                slidesLeftExtend += .1;
                slidesRightExtend -= .1;

                slidesLeftExtend = Range.clip(slidesLeftExtend, params.SLIDE_LEFT_EXTEND, params.SLIDE_LEFT_RESET);
                slidesRightExtend = Range.clip(slidesRightExtend, params.SLIDE_RIGHT_RESET, params.SLIDE_RIGHT_EXTEND);

                mechOps.slidesCustomExtension(slidesLeftExtend, slidesRightExtend);
            }

            if(gamepad2.x) {

                if(fourBarPosition == FourBarPosition.FOUR_BAR_IN || fourBarPosition == FourBarPosition.FOUR_BAR_MID) {
                    mechOps.clawRightClose();
                    mechOps.clawleftclose();
                    rightClawOpen = false;
                    leftClawOpen = false;
                    sleep(20);
                    mechOps.armExtend();
                    mechOps.slidesReset(); // will be changed by driver
                    mechOps.wristPosition(params.WRIST_EXTEND);
                    fourBarPosition = FourBarPosition.FOUR_BAR_OUT;
                    elapsedTimeOut.reset();
                }


            } else if(gamepad2.b) {
                mechOps.clawRightClose();
                mechOps.clawleftclose();
                rightClawOpen = false;
                leftClawOpen = false;
                mechOps.slidesReset();
                mechOps.wristPosition(params.WRIST_LOAD_PIXELS);
                sleep(20);
                fourBarPosition = FourBarPosition.FOUR_BAR_IN;
                mechOps.armReset();
                slidesLeftExtend = params.SLIDE_LEFT_RESET;
                slidesRightExtend = params.SLIDE_RIGHT_RESET;
                elapsedTimeIn.reset();
//                passthroughMode = true;
            }
            if (gamepad2.right_stick_button) {
                mechOps.clawleftclose();
                mechOps.clawRightClose();
                rightClawOpen = false;
                leftClawOpen = false;
            }

            if(gamepad2.left_stick_button) {
                mechOps.clawleftclose();
                mechOps.clawRightClose();
                sleep(20);
                mechOps.armIdle();
            }

            if(elapsedTimeIn.time() > 1 && elapsedTimeIn.time() < 1.5 ) {
                if(!passthroughMode && fourBarPosition == FourBarPosition.FOUR_BAR_IN) {
                    mechOps.clawRightOpenBucket();
                    mechOps.clawleftopenBucket();
                } else {
                    mechOps.clawleftclose();
                    mechOps.clawRightClose();
                }
            }

            if(elapsedTimeOut.time() > 1 && elapsedTimeOut.time() < 1.5 ) {
                if(!passthroughMode && fourBarPosition == FourBarPosition.FOUR_BAR_OUT) {
                    mechOps.clawRightOpenBucket();
                    mechOps.clawleftopenBucket();
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

            if(gamepad2.y) {
                mechOps.bucketScore();
                mechOps.fingerHoldRight();
                mechOps.fingerHoldLeft();
            } else if(gamepad2.a) {
                mechOps.bucketReset();
                mechOps.fingerReleaseRight();
                mechOps.fingerReleaseLeft();
            }
            /*---------------DRONE CONTROL---------------*/
            if(gamepad1.a) {
                mechOps.droneFire();
            } else if(gamepad1.b) {
                mechOps.droneLoad();
            }

            robot.droneActuator.setPower(0);

            if(gamepad1.dpad_left && !mechOps.bucketScored) {
                robot.droneActuator.setPower(.25);
                robot.droneActuator.setDirection(DcMotorSimple.Direction.FORWARD);
            } else if(gamepad1.dpad_right  && !mechOps.bucketScored) {
                robot.droneActuator.setPower(.25);
                robot.droneActuator.setDirection(DcMotorSimple.Direction.REVERSE);

            }

            /*---------------LIFT CONTROL---------------*/

            if(gamepad2.dpad_right) {

                if(fourBarPosition == FourBarPosition.FOUR_BAR_IN) {
                    passthroughMode = true;
                    mechOps.fingerHoldLeft();
                    mechOps.fingerHoldRight();
                    elapsedTime.reset();
                } else {
                    liftPos = params.LIFT_HIGH_POSITION;
                    mechOps.liftPosition(params.LIFT_HIGH_POSITION);
                }

            } else if(gamepad2.dpad_left) {
                mechOps.bucketReset();
                if(fourBarPosition != FourBarPosition.FOUR_BAR_IN) {
                    liftPos = 0;
                    mechOps.liftPosition(0);
                }
            }

            if(gamepad2.dpad_up) {
                climbModeTime.reset();
                climbMode = true;
                mechOps.clawleftclose();
                mechOps.clawRightClose();
//                mechOps.liftPosition(params.LIFT_MAX_HEIGHT);
//                mechOps.clawleftclose();
//                mechOps.clawRightClose();
                mechOps.armIdle();
//                sleep(20);
//                mechOps.bucketScore();
            } else if(gamepad2.dpad_down) {
                climbMode = false;
                mechOps.liftPosition(params.LIFT_MID_POSITION);
                mechOps.clawleftclose();
                mechOps.clawRightClose();
//                mechOps.armReset();
            }

            if(climbMode == true) {
                if(climbModeTime.time() > .5 && climbModeTime.time() < 1) {
                    mechOps.armIdle();
                    mechOps.bucketScore();
                    mechOps.liftPos(params.LIFT_MAX_HEIGHT);
                }

                if(robot.motorLift.getCurrentPosition() >= params.LIFT_HIGH_POSITION) {
                    mechOps.clawleftclose();
                    mechOps.clawRightClose();
                    sleep(20);
                    mechOps.armReset();
                    mechOps.slidesReset();
                }
            }

            if(gamepad1.dpad_up) {
                mechOps.fingerOut();
            } else if(gamepad1.dpad_down) {
                mechOps.fingerIn();
            }


            if(gamepad2.right_trigger > .1 ) {
                liftPos += 100;
                mechOps.liftPosition(liftPos);
            } else if(gamepad2.left_trigger > .1) {
                liftPos -= 100;
                mechOps.liftPosition(liftPos);
            }

            if(passthroughMode) {
                if(elapsedTime.time() > .5 && elapsedTime.time() < 1) {
                    rightClawOpen = false;
                    leftClawOpen = false;
//                    mechOps.wristPosition(1);
                    mechOps.armIdle();
//                    mechOps.clawRightClose();
//                    mechOps.clawleftclose();

                    fourBarPosition = FourBarPosition.FOUR_BAR_MID;
                } else if(elapsedTime.time() > 1 && elapsedTime.time() < 1.5) {
                    liftPos = params.LIFT_MID_POSITION;
                    mechOps.liftPosition(params.LIFT_MID_POSITION);
                    mechOps.bucketScore();
                    mechOps.fingerHoldRight();
                    mechOps.fingerHoldLeft();
                    mechOps.clawRightClose();
                    mechOps.clawleftclose();
                } else if(elapsedTime.time() > 2) {
                    mechOps.wristPosition(params.WRIST_EXTEND);
                    passthroughMode = false;
                }
            }

            mechOps.leftLEDState(!leftClawOpen);
            mechOps.rightLEDState(!rightClawOpen);

            // 90 degree turn

            // Provide user feedback
            telemetry.addData("drone pos = ", dronePos);
            telemetry.addData("finger right", robot.servoBucketFingerRight.getPosition());
            telemetry.addData("sensor left debounce", sensorLeftDebounce.time());
            telemetry.addData("sensor right debounce", sensorRightDebounce.time());
            telemetry.addData("claw sensor right color", robot.clawSensorRight.getLightDetected());
            telemetry.addData("claw sensor left color", robot.clawSensorLeft.getLightDetected());
            telemetry.addData("claw sensor right distance", robot.clawSensorRight.getDistance(DistanceUnit.MM));
            telemetry.addData("claw sensor left distance", robot.clawSensorLeft.getDistance(DistanceUnit.MM));
            telemetry.addData("slides left extend", slidesLeftExtend);
            telemetry.addData("slides right extend", slidesRightExtend);
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
            telemetry.update();

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}       // end of MSTeleop class