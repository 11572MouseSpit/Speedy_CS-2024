package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.Hardware.RRHWProfile;
import org.firstinspires.ftc.teamcode.Libs.RRMechOps;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Driver TeleOp", group = "Competition")

public class DriverControlled extends LinearOpMode {
    private final static RRHWProfile robot = new RRHWProfile();
    private final static Params params = new Params();
    private static int liftHoldPos = 0;
    private static boolean holdLift = false;
    private static final boolean debug = false  ;
    private static Thread droneShootThread = new Thread() ;
    private static int liftPos = 0;
    public boolean climbIdle = false;
    public boolean armHomed = false;
    private MecanumDrive driveBase;
    GamepadEx driverOp;


    @Override
    public void runOpMode(){
        boolean ftcLibEnabled = false;
        double v1 = 0, v2 = 0, v3 = 0, v4 = 0, robotAngle;
        double theta;
        double theta2 = 180;
        int flippedHeadingAngle = 110;
        double r;
        double power = 1;
        double rightX, rightY;
        double TargetRotation = 0;
        double OldRotation = 0;
        boolean canOpen = true;
        boolean intakeMode = false;
        boolean armHoverEnabled = true;
        boolean rotateEnabled = false;
        boolean fieldCentric = true;
        FourBarPosition fourBarPosition = FourBarPosition.FOUR_BAR_IN;
        int targetPosition = 0;
        LinearOpMode opMode = this;
        boolean passthroughMode = false;
//        double liftPower = robot.LIFT_POWER_DOWN;
        ElapsedTime elapsedTime = new ElapsedTime();
        ElapsedTime elapsedTimeIn = new ElapsedTime();
        ElapsedTime shareHoldTime = new ElapsedTime();
        ElapsedTime responseTime = new ElapsedTime();
        ElapsedTime elapsedTimeOut = new ElapsedTime();
        int clawAxisPos = 0;
        boolean leftClawOpen = false;
        boolean rightClawOpen = false;
        boolean rbCooldown = false;
        boolean dpadRightCooldown = false;
        boolean dpadUpBtnCooldown = false;
        boolean dpadDownBtnCooldown = false;
        boolean yBtnCooldown = false;
        boolean bBtnCooldown = false;
        boolean rtBtnReleased = false;
        boolean shareCooldown = false;
        boolean lbCooldown = false;
        boolean aCooldown = false;
        boolean climbMode = false;
        ElapsedTime climbModeTime = new ElapsedTime();
        ElapsedTime clawLeftAutoWait = new ElapsedTime();
        ElapsedTime clawRightAutoWait = new ElapsedTime();
        boolean slowMode = false;
        double RFrotatePower = params.TURN_SPEED;
        double slidesLeftExtend = params.SLIDE_LEFT_EXTEND;
        double slidesRightExtend = params.SLIDE_RIGHT_EXTEND;
        double LFrotatePower = -params.TURN_SPEED;
        double LRrotatePower = -params.TURN_SPEED;
        boolean clawLeftHasPixel = false;
        double imuValue = 0;
        boolean clawRightHasPixel = false;
        boolean firstRun = true;
        boolean climbLocked = true;
        double RRrotatePower = params.TURN_SPEED;
        double extraPower = 1.428571429;
        ElapsedTime sensorLeftDebounce = new ElapsedTime();
        ElapsedTime sensorRightDebounce = new ElapsedTime();
        int dronePos = 0;

        robot.init(hardwareMap, true);

        RRMechOps mechOps = new RRMechOps(robot, opMode, params);

//        MecanumDrive beans = new MecanumDrive(hardwareMap, new Pose2d(0,0,Math.toRadians(0)));
//        beans.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Ready to Run: ","GOOD LUCK");
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();

        params.scoreHigh = false;
        params.armDeployed = false;
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(!opModeIsActive()) {
            if(isStopRequested()) {
                break;
            }
        }

        driveBase = new MecanumDrive(robot.ftcLib_motorLF, robot.ftcLib_motorRF, robot.ftcLib_motorLR, robot.ftcLib_motorRR);
        driveBase.setRightSideInverted(false);
        driverOp = new GamepadEx(gamepad1);

        new Thread(() -> {
            while (true) {
                if (climbIdle) {
                    robot.climbMotorRight.setPower(-params.IDLE_CLIMB_DOWN);
                    robot.climbMotorLeft.setPower(-params.IDLE_CLIMB_DOWN);
                }
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        });

        while (opModeIsActive()) {

            /*******************************************
             ****** Mecanum Drive Control section ******
             *******************************************/

            // reset imu
            if(gamepad1.options){
                robot.imu2.init();
                gamepad1.rumble(500);
            }

            responseTime.reset();

            imuValue = robot.imu2.getAbsoluteHeading();

            if(firstRun) {
//                if(!mechOps.homed) {
////                    mechOps.homeArm();
//                    mechOps.homed = true;
//                }

                new Thread(() -> {
                    try {
                        armHomed = false;
                        mechOps.armCustomPos(-600);
                        Thread.sleep(1500);
                        robot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        while (true) {
                            robot.armMotor.setPower(.35);
                            if(robot.armMotor.getCurrent(CurrentUnit.AMPS) > 3) {
                                armHomed = true;
                                robot.armMotor.setPower(0);
                                break;
                            }
                        }
                        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.armMotor.setTargetPosition(0);
                        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armHomed = true;

                        gamepad1.rumble(250);
                        Thread.sleep(250);
                        gamepad1.rumble(250);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }).start();
                robot.imu2.init();
                mechOps.climbLock();
                mechOps.droneLoad();
                mechOps.clawDown();
                mechOps.armDown();
                mechOps.intakeRetract();

                firstRun = false;
                clawLeftAutoWait.startTime();
                clawRightAutoWait.startTime();

                mechOps.homeClimb();
            }

            if(ftcLibEnabled) {
//                beans.setDrivePowers(new PoseVelocity2d(
//                        new Vector2d(
//                                -(gamepad1.left_stick_y*Math.cos(Math.toRadians(theta))+gamepad1.left_stick_x*Math.sin(Math.toRadians(theta)))*1,
//                                -(gamepad1.left_stick_x*Math.cos(Math.toRadians(theta))-gamepad1.left_stick_y*Math.sin(Math.toRadians(theta)))*1
//                        ),
//                        -gamepad1.right_stick_x
//                ));

                //beans.updatePoseEstimate();

                if(gamepad1.left_stick_button || params.armDeployed) {
                    driveBase.setMaxSpeed(params.SLOW_MODE_SPEED);
                } else {
                    driveBase.setMaxSpeed(params.NORMAL_SPEED);
                }
                driveBase.driveFieldCentric(
                        -driverOp.getLeftX(),
                        -driverOp.getLeftY(),
                        -driverOp.getRightX(),
                        imuValue
                );
            } else {

                if (fieldCentric) {             // verify that the user hasn't disabled field centric drive
                    theta =  imuValue + 0;
//                        robot.imu.getAngularOrientation().firstAngle + 90;
                } else {
                    theta = 0;      // do not adjust for the angular position of the robot
                }

                if(gamepad1.left_stick_button || params.armDeployed) {
                    power = params.SLOW_MODE_SPEED;
                } else {
                    power = params.NORMAL_SPEED;
                }

                robotAngle = Math.atan2(-gamepad1.left_stick_y, (gamepad1.left_stick_x)) - Math.PI / 4;
                rightX = gamepad1.right_stick_x;
                rightY = gamepad1.right_stick_y;


                r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);

                v1 = (r * Math.cos(robotAngle - Math.toRadians(theta + theta2)) - rightX + rightY) * extraPower;
                v2 = (r * Math.sin(robotAngle - Math.toRadians(theta + theta2)) + rightX + rightY) * extraPower;
                v3 = (r * Math.sin(robotAngle - Math.toRadians(theta + theta2)) - rightX + rightY) * extraPower;
                v4 = (r * Math.cos(robotAngle - Math.toRadians(theta + theta2)) + rightX + rightY) * extraPower;

                if (((imuValue - OldRotation >= TargetRotation && TargetRotation > 0) || (imuValue - OldRotation <= TargetRotation && TargetRotation < 0)) && rotateEnabled) {
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
            }

            clawLeftHasPixel = mechOps.clawHasPixel(robot.clawSensorLeft);
            clawRightHasPixel = mechOps.clawHasPixel(robot.clawSensorRight);

            // ARM CONTROL

            if(armHomed) {

                if (armHoverEnabled) {
                    if (!params.armDeployed && !intakeMode && !params.armResetThread.isAlive() && !droneShootThread.isAlive()) {
                        mechOps.armCustomPosAndPower(params.ARM_FLOAT_POS, params.ARM_LOW_POWER);
                    } else if (intakeMode && !params.armDeployed && !params.armResetThread.isAlive() && !droneShootThread.isAlive()) {
                        mechOps.armDown();
                    }
                }

                if (gamepad2.share && !shareCooldown) {
                    shareCooldown = true;
                    climbIdle = true;
                }

                if (!gamepad2.share) {
                    shareCooldown = false;
                }

                if (!gamepad1.share) {
                    shareHoldTime.reset();
                } else if (shareHoldTime.time(TimeUnit.MILLISECONDS) > 1500) {
                    gamepad1.rumble(500);
                    new Thread(() -> {
                        if (!params.armDeployed) {
                            robot.armMotor.setTargetPosition(0);
                            robot.armMotor.setPower(0);
                            try {
                                Thread.sleep(500);
                            } catch (InterruptedException e) {
                                throw new RuntimeException(e);
                            }
                            robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        }
                    });
                }

                if (gamepad1.a && !aCooldown) {
                    aCooldown = true;
                    if (params.armDeployed) {
                        params.armDeployed = false;
                        params.armResetThread = new Thread(() -> {
                            try {
                                mechOps.armHighScore();
                                mechOps.clawDown();
                                params.armDeployed = false;
                                params.scoreHigh = false;
                                Thread.sleep(250);
                                mechOps.clawLeftClose();
                                mechOps.clawRightClose();
                                liftHoldPos = 0;
                                holdLift = true;
                                liftPos = mechOps.liftPosition(0);
                                while (robot.motorLift.getCurrentPosition() > 20) {
                                    Thread.sleep(250);
                                }
                                ;
                                if (!Thread.interrupted()) {
                                    mechOps.clawOffGround();
                                    mechOps.armCustomPos(params.ARM_RESET_POS);
                                }
                                while (robot.armMotor.getCurrentPosition() > Params.ARM_RESET_POS + 50) {
                                    Thread.sleep(250);
                                }
                                Thread.sleep(750);
                                ;
                                double oldArmMotorPower = params.ARM_IDLE_POWER;
                                params.ARM_IDLE_POWER = .3;
                                Thread.sleep(300);//interrupted() is checked twice because the
                                // thread can be interrupted during the sleep
                                if (!Thread.interrupted()) {
                                    mechOps.armDown();
                                    mechOps.clawDown();
                                }
                                params.ARM_IDLE_POWER = oldArmMotorPower;
                                Thread.sleep(250);//interrupted() is checked twice because the
                                // thread can be interrupted during the sleep
                                if (!Thread.interrupted()) {
                                }

                                params.scoreHigh = false;
                            } catch (InterruptedException e) {
                                // do nothing
                            }
                        });
                        params.armResetThread.start();
                    } else {
                        params.armDeployed = true;
                        if (params.armResetThread.isAlive()) {
                            params.armResetThread.interrupt();
                        }
                        new Thread(() -> {
                            try {
                                mechOps.clawLeftClose();
                                mechOps.clawRightClose();
                                Thread.sleep(200);
                                if (!Thread.interrupted()) {
                                    mechOps.armUpMid();
                                }
                                while (robot.armMotor.getCurrentPosition() < params.ARM_MIDWAY_UP - 150) {
                                    Thread.sleep(250);
                                }
                                mechOps.armUp();
                                mechOps.clawUp();
                                liftHoldPos = 0;
                                holdLift = true;
                                liftPos = mechOps.liftPosition(0);
                            } catch (InterruptedException e) {
                                // do nothing
                            }
                        }).start();
                    }
                }

                if (gamepad1.b) {
                    if (params.armDeployed == true) {
                        holdLift = true;
                        liftHoldPos = params.LIFT_MAX_HEIGHT;
                        liftPos = mechOps.liftPosition(params.LIFT_MAX_HEIGHT);
                    }
                }

                if (!gamepad1.a) aCooldown = false;

                //CLAW CONTROL

                if (gamepad1.right_trigger > .1 && !params.armDeployed) {
                    intakeMode = true;
                    if (params.pickupStack) {
                        mechOps.intakeDeployStack();
                    } else {
                        mechOps.intakeDeploy();
                    }
                    rtBtnReleased = true;
                    if (!clawLeftHasPixel) mechOps.clawLeftOpen();
                    if (!clawRightHasPixel) mechOps.clawRightOpen();
                } else if (rtBtnReleased == true) {
                    intakeMode = false;
                    mechOps.intakeRetract();
                    mechOps.clawLeftClose();
                    mechOps.clawRightClose();
                    rtBtnReleased = false;
                }

                if (gamepad1.dpad_up && !dpadUpBtnCooldown && params.armDeployed) {
                    dpadUpBtnCooldown = true;
                    mechOps.clawFinger();
                }

                if (!gamepad1.dpad_up) dpadUpBtnCooldown = false;

                if (gamepad1.dpad_down && !dpadDownBtnCooldown && params.armDeployed) {
                    dpadDownBtnCooldown = true;
                    mechOps.clawUp();
                }

                if (!gamepad1.dpad_up) dpadUpBtnCooldown = false;
                if (!gamepad1.dpad_down) dpadDownBtnCooldown = false;

                if (clawLeftHasPixel && !params.armDeployed) {
                    if (clawLeftAutoWait.time(TimeUnit.MILLISECONDS) > params.CLAW_AUTO_GRAB_COOLDOWN) {
                        mechOps.clawLeftClose();
                        clawLeftAutoWait.reset();
//                    gamepad1.rumble(500, 0, 500);
                    }
                }

                if (clawRightHasPixel && !params.armDeployed) {
                    if (clawRightAutoWait.time(TimeUnit.MILLISECONDS) > params.CLAW_AUTO_GRAB_COOLDOWN) {
                        mechOps.clawRightClose();
                        clawRightAutoWait.reset();
//                    gamepad1.rumble(0, 500, 500);
                    }
                }

                if (gamepad1.b && !bBtnCooldown) {
                    bBtnCooldown = true;

                    if (!params.armDeployed) {
                        mechOps.clawLeftClose();
                        mechOps.clawRightClose();
                    }
                }

                if (gamepad1.y && !yBtnCooldown) {
                    yBtnCooldown = true;

                    if (!params.armDeployed) {
                        mechOps.clawLeftOpen();
                        mechOps.clawRightOpen();
                    } else {
                        mechOps.clawLeftOpenBoard();
                        mechOps.clawRightOpenBoard();
                    }
                }

                if (!gamepad1.y) yBtnCooldown = false;
                if (!gamepad1.b) bBtnCooldown = false;

                if (gamepad1.right_bumper && !rbCooldown) {
                    rbCooldown = true;
                    if (!rightClawOpen) {
                        if (params.armDeployed) {
                            mechOps.clawLeftOpenBoard();
                        } else {
                            // if flipped
                            if (imuValue > flippedHeadingAngle || imuValue < -flippedHeadingAngle) {
                                mechOps.clawLeftOpen();
                            } else {
                                mechOps.clawRightOpen();
                            }
                        }
                        rightClawOpen = true;
                    } else {
                        if (params.armDeployed) {
                            mechOps.clawLeftClose();
                        } else {
                            // if flipped
                            if (imuValue > flippedHeadingAngle || imuValue < -flippedHeadingAngle) {
                                mechOps.clawLeftClose();
                            } else {
                                mechOps.clawRightClose();
                            }
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
                            mechOps.clawRightOpenBoard();
                        } else {
                            // if flipped
                            if (imuValue > flippedHeadingAngle || imuValue < -flippedHeadingAngle) {
                                mechOps.clawRightOpen();
                            } else {
                                mechOps.clawLeftOpen();
                            }
                        }
                        leftClawOpen = true;
                    } else {
                        if (params.armDeployed) {
                            mechOps.clawRightClose();
                        } else {
                            // if flipped
                            if (imuValue > flippedHeadingAngle || imuValue < -flippedHeadingAngle) {
                                mechOps.clawRightClose();
                            } else {
                                mechOps.clawLeftClose();
                            }
                        }
                        leftClawOpen = false;
                    }
                }

                if (!gamepad1.left_bumper) {
                    lbCooldown = false;
                }
            }

            // LIFT CONTROL

            if (params.armDeployed) {
                if(gamepad1.x || gamepad1.right_stick_button) {
                    //mechOps.liftPositionNoReturn(params.LIFT_MID_POSITION);
                    //mechOps.armCustomPos(params.ARM_ABOVE_BLOCK);
                } else {
                    if (gamepad1.right_trigger > .1) {
                        holdLift = false;
                        liftPos = mechOps.liftPosition(liftPos + 500);
                    } else if (gamepad1.left_trigger > .1) {
                        holdLift = false;
                        liftPos = mechOps.liftPosition(liftPos - 500);
                    } else {
                        if (holdLift == false) {
                            liftHoldPos = robot.motorLift.getCurrentPosition();
                            holdLift = true;
                        }
                        liftPos = mechOps.liftPosition(liftHoldPos);
                    }
                }
            }

            if(gamepad1.dpad_left && debug) {
                params.pickupStack = true;
                mechOps.armCustomPos(params.ARM_STACK_5_POS);
            }

            if(gamepad1.dpad_right && debug) {
                params.pickupStack = false;
                new Thread(() -> {
                   if(params.pickupStack = true) {
                       mechOps.armHalfwayDown();
                       try {
                           Thread.sleep(2000);
                       } catch (InterruptedException e) {
                           throw new RuntimeException(e);
                       }
                       mechOps.armDown();
                   }
                });
            }

            // CLIMB CONTROL

            if(gamepad1.dpad_right && !dpadRightCooldown) {
                if(climbLocked) {
                    mechOps.climbUnlock();
                    climbLocked = false;
                } else {
                    mechOps.climbLock();
                    climbLocked = true;
                }
                dpadRightCooldown = true;
            }

            if(!gamepad1.dpad_right) dpadRightCooldown = false;

            if(gamepad2.b) {
                mechOps.climbUnlock();
            } else if(gamepad2.y) {
                mechOps.climbLock();
            }

            if(gamepad2.right_stick_y > .1) {
                robot.climbMotorRight.setPower(-.6);
            } else if(gamepad2.right_stick_y < -.1) {
                robot.climbMotorRight.setPower(.6);
            } else if(gamepad2.left_stick_y > .1) {
                robot.climbMotorLeft.setPower(-.6);
            } else if(gamepad2.left_stick_y < -.1) {
                robot.climbMotorLeft.setPower(.6);
            } else {
                if (gamepad1.dpad_up || gamepad2.dpad_up) {
                    if(!climbLocked) mechOps.climbUp();
                } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
                    if(!climbLocked)   mechOps.climbDown();
                } else {
                    robot.climbMotorLeft.setPower(0);
                    robot.climbMotorRight.setPower(0);
                }
            }

            // DRONE

            if(gamepad2.a || gamepad1.dpad_left) {
                droneShootThread = new Thread(() -> {
                    mechOps.armCustomPos(params.DRONE_FIRE_ARM_POS);
                    try {
                        Thread.sleep(1500);
                        mechOps.droneFire();
                        Thread.sleep(750);
                        mechOps.armDown();
                    } catch (InterruptedException e) {
                    }
                });
                droneShootThread.start();
            }

            mechOps.clawRightLedState(clawRightHasPixel);
            mechOps.clawLeftLedState(clawLeftHasPixel);

//            telemetry.addData("robot x", beans.pose.position.x);
//            telemetry.addData("robot y", beans.pose.position.y);
//            telemetry.addData("robot heading", beans.pose.heading.toDouble());
            telemetry.addData("Arm Current: ", robot.armMotor.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Arm Homed: ", mechOps.homed);
            telemetry.addData("Arm Encoder Pos: ", robot.armMotor.getCurrentPosition());
            telemetry.addData("response time: ", responseTime.time(TimeUnit.MILLISECONDS));
            telemetry.addData("v1", v1);
            telemetry.addData("v2", v2);
            telemetry.addData("v3", v3);
            telemetry.addData("v4", v4);
            telemetry.addData("Arm Speed Pos: ", robot.armMotor.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("Lift Encoder Pos: ", robot.motorLift.getCurrentPosition());
            telemetry.addData("Arm Current: ", robot.armMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Lift Current: ", robot.motorLift.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Climb Left Current: ", robot.climbMotorLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Climb Right Current: ", robot.climbMotorRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("motorLR current: ", robot.motorLR.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("motorLF current: ", robot.motorLF.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("motorRR current: ", robot.motorRR.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("motorRF current: ", robot.motorRF.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("left claw open: ", leftClawOpen);
            telemetry.addData("right claw open: ", rightClawOpen);
            telemetry.addData("right claw sense distance: ", robot.clawSensorRight.getDistance(DistanceUnit.CM));
            telemetry.addData("left claw sense distance: ", robot.clawSensorLeft.getDistance(DistanceUnit.CM));
            telemetry.addData("robot imu: ", imuValue     );
            telemetry.addData("clawRightAutoWait time: ", clawRightAutoWait.time(TimeUnit.MILLISECONDS));
            telemetry.update();
        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}       // end of MSTeleop class