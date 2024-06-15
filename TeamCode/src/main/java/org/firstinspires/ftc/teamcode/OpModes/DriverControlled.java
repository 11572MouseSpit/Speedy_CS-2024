package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.Hardware.RRHWProfile;
import org.firstinspires.ftc.teamcode.Libs.RRMechOps;

@TeleOp(name = "Driver TeleOp", group = "Competition")

public class DriverControlled extends LinearOpMode {
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
        boolean aCooldown = false;
        int liftHoldPos = 0;
        boolean holdLift = false;
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

        RRMechOps mechOps = new RRMechOps(robot, opMode, params);

        telemetry.addData("Ready to Run: ","GOOD LUCK");
        telemetry.update();

        params.scoreHigh = false;
        params.armDeployed = false;
//        mechOps.clawDown();
//        robot.armMotor.setPower(.8);
        robot.armMotor.setTargetPosition(0);
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mechOps.armRetract();

        while(!opModeIsActive()) {
            if(isStopRequested()) {
                break;
            }
        }

        while (opModeIsActive()) {

            /*******************************************
             ****** Mecanum Drive Control section ******
             *******************************************/
            if (fieldCentric) {             // verify that the user hasn't disabled field centric drive
                theta = robot.imu.getAbsoluteHeading() + 0;
//                        robot.imu.getAngularOrientation().firstAngle + 90;
            } else {
                theta = 0;      // do not adjust for the angular position of the robot
            }

            robotAngle = Math.atan2(-gamepad1.left_stick_y, (gamepad1.left_stick_x)) - Math.PI / 4;
            rightX = gamepad1.right_stick_x;
            rightY = gamepad1.right_stick_y;


            r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y) * 1;
            if (slowMode == 1) {
                rightX *= params.SLOW_TURN_SPEED;
                rightY *= params.SLOW_TURN_SPEED;
                r *= params.SLOW_MOVE_SPEED;
            } else if (slowMode == 2) {
                rightX *= params.SLOW_TURN_SPEED;
                rightY *= params.SLOW_TURN_SPEED;
                r *= params.SLOWEST_MOVE_SPEED;
            }

            v1 = (r * Math.cos(robotAngle - Math.toRadians(theta + theta2)) - rightX + rightY);
            v2 = (r * Math.sin(robotAngle - Math.toRadians(theta + theta2)) + rightX + rightY);
            v3 = (r * Math.sin(robotAngle - Math.toRadians(theta + theta2)) - rightX + rightY);
            v4 = (r * Math.cos(robotAngle - Math.toRadians(theta + theta2)) + rightX + rightY);

            if (((robot.imu.getAbsoluteHeading() - OldRotation >= TargetRotation && TargetRotation > 0) || (robot.imu.getAbsoluteHeading() - OldRotation <= TargetRotation && TargetRotation < 0)) && rotateEnabled) {
                rotateEnabled = false;
                TargetRotation = 0;
                OldRotation = 0;
            }

            if (!rotateEnabled) {
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

            // ARM CONTROL

            if (gamepad1.a && !aCooldown) {
                aCooldown = true;
                if (params.armDeployed) {
                    params.armDeployed = false;
                    params.armResetThread = new Thread(() -> {
                        try {
                            params.scoreHigh = true;
                            mechOps.liftPosition(0);
                            while(robot.motorLift.getCurrentPosition() > 20) continue;
                            params.scoreHigh = false;
                            if (!Thread.interrupted()) {
                                mechOps.armHalfwayExtend();
                                mechOps.clawDown();
                                mechOps.armHalfwayDown();
                            }
                            Thread.sleep(2000);//interrupted() is checked twice because the
                            // thread can be interrupted during the sleep
                            if (!Thread.interrupted()) {
                                mechOps.armDown();
                            }
                            Thread.sleep(250);//interrupted() is checked twice because the
                            // thread can be interrupted during the sleep
                            if (!Thread.interrupted()) {
                                mechOps.armRetract();
                            }

                            params.scoreHigh = false;
                        } catch (InterruptedException e) {
                            // do nothing
                        }
                    });
                    params.armResetThread.start();

                    liftPos = robot.motorLift.getCurrentPosition();
                } else {
                    params.armDeployed = true;
                    if (params.armResetThread.isAlive()) {
                        params.armResetThread.interrupt();
                    }
                    new Thread(() -> {
                        try {
                            mechOps.armHalfwayExtend();
                            Thread.sleep(500);
                            if (!Thread.interrupted()) {
                                mechOps.armUpMid();
                            }
                            while(robot.armMotor.getCurrentPosition() < params.ARM_MIDWAY_UP - 50) continue;
                            mechOps.armRetract();
                            mechOps.clawUp();
                            Thread.sleep(500);
                            mechOps.armUp();
                        } catch (InterruptedException e) {
                            // do nothing
                        }
                    }).start();
                }
            }

            if (gamepad1.b) {
                if (params.armDeployed == true) {
                    mechOps.armHighScore();
                    mechOps.clawUpHighScore();
                    liftPos = mechOps.liftPosition(params.LIFT_MAX_HEIGHT);
                }
            }

            if (!gamepad1.a) aCooldown = false;

            //CLAW CONTROL

            if (gamepad1.right_bumper && !rbCooldown) {
                rbCooldown = true;
                if (!rightClawOpen) {
                    if (params.armDeployed) {
                        mechOps.clawLeftOpenBucket();
                    } else {
                        mechOps.clawRightOpen();
                    }
                    rightClawOpen = true;
                } else {
                    if(params.armDeployed) {
                        mechOps.clawleftclose();
                    } else {
                        mechOps.clawRightClose();
                    }
                    rightClawOpen = false;
                }
            }

            if (!gamepad1.right_bumper) {
                rbCooldown = false;
            }

            if (gamepad1.left_bumper && !lbCooldown) {
                lbCooldown = true;
                if (!leftClawOpen) {
                    if (params.armDeployed) {
                        mechOps.clawRightOpenBucket();
                    } else {
                        mechOps.clawLeftOpen();
                    }
                    leftClawOpen = true;
                } else {
                    if(params.armDeployed) {
                        mechOps.clawRightClose();
                    } else {
                        mechOps.clawleftclose();
                    }
                    leftClawOpen = false;
                }
            }

            if (!gamepad1.left_bumper) {
                lbCooldown = false;
            }

            // EXTEND CONTROL

            if (gamepad1.x) {
                mechOps.armExtend();
            }
            if (gamepad1.y) {
                mechOps.armRetract();
            }

            // LIFT CONTROL

            if (!params.scoreHigh) {
                if (gamepad1.right_trigger > .1) {
                    holdLift = false;
                    liftPos = mechOps.liftPosition(liftPos + 100);
                } else if (gamepad1.left_trigger > .1) {
                    holdLift = false;
                    liftPos = mechOps.liftPosition(liftPos - 100);
                }

                if (gamepad1.right_trigger < .1 && gamepad1.left_trigger < .1) {
                    if(holdLift == false) {
                        liftHoldPos = robot.motorLift.getCurrentPosition();
                        holdLift = true;
                    }
                    liftPos = mechOps.liftPosition(liftHoldPos);
                }
            }

            telemetry.addData("Arm Encoder Pos: ", robot.armMotor.getCurrentPosition());
            telemetry.addData("Lift Encoder Pos: ", robot.motorLift.getCurrentPosition());
            telemetry.addData("Lift Current: ", robot.motorLift.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("motorLR current: ", robot.motorLR.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("motorLF current: ", robot.motorLF.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("motorRR current: ", robot.motorRR.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("motorRF current: ", robot.motorRF.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("left claw open: ", leftClawOpen);
            telemetry.addData("right claw open: ", rightClawOpen);
            telemetry.update();
        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}       // end of MSTeleop class