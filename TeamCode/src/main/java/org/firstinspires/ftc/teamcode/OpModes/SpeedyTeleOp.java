package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveClass;

@TeleOp(name = "Field Centric", group = "Competition")

public class SpeedyTeleOp extends LinearOpMode {
    private final static HWProfile robot = new HWProfile();

    @Override
    public void runOpMode(){
        double v1, v2, v3, v4, robotAngle;
        double theta;
        double theta2 = 180;
        double r;
        double power = 1;
        double rightX, rightY;
        boolean fieldCentric = true;
        boolean passthroughMode = false;
        int targetPosition = 0;
        LinearOpMode opMode = this;
        double liftPower = robot.LIFT_POWER_DOWN;
        double liftPos = 0;
        boolean bRefresh = false;
        int clawAxisPos = 0;
        boolean clawPos = false;
        ElapsedTime elapsedTime = new ElapsedTime();
        ElapsedTime lastButtonPress = new ElapsedTime();
        boolean aRefresh = false;
        int thetaPos = 90;
        boolean thetaMode = false;


        robot.init(hardwareMap);

        DriveClass drive = new DriveClass(robot, opMode);

        telemetry.addData("Ready to Run: ","GOOD LUCK");
        telemetry.update();

        robot.launcherServo.setPosition(0);

        waitForStart();

        robot.clawAxis.setPosition(1);

if(passthroughMode) {
                if(elapsedTime.time() > 1 && elapsedTime.time() < 1.5) {
                    robot.launcherServo.setPosition(1);
                    robot.bucketAxisServo.setPosition(.45);
//                    passthroughMode = false;
                } else if(elapsedTime.time() > 2.2 && elapsedTime.time() < 2.5) {
                    drive.openClaw();
//                    passthroughMode = false;
                } else if(elapsedTime.time() > 2.5 && elapsedTime.time() < 3) {
                    robot.launcherServo.setPosition(.4);
                    passthroughMode = false;
                }
            }
        while (opModeIsActive()) {

            /*******************************************
             ****** Mecanum Drive Control section ******
             *******************************************/
            if (fieldCentric) {             // verify that the user hasn't disabled field centric drive
                theta = robot.imu.getAbsoluteHeading() + thetaPos;
//                        robot.imu.getAngularOrientation().firstAngle + 90;
            } else {
                theta = 0;      // do not adjust for the angular position of the robot
            }



            robotAngle = Math.atan2(gamepad1.left_stick_y, (-gamepad1.left_stick_x)) - Math.PI / 4;
            rightX = -gamepad1.right_stick_x;
            rightY = -gamepad1.right_stick_y;
            r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);

            v1 = (r * Math.cos(robotAngle - Math.toRadians(theta + theta2)) - rightX + rightY);
            v2 = (r * Math.sin(robotAngle - Math.toRadians(theta + theta2)) + rightX + rightY);
            v3 = (r * Math.sin(robotAngle - Math.toRadians(theta + theta2)) - rightX + rightY);
            v4 = (r * Math.cos(robotAngle - Math.toRadians(theta + theta2)) + rightX + rightY);

            robot.motorLF.setPower(Range.clip((v1), -power, power));
            robot.motorRF.setPower(Range.clip((v2), -power, power));
            robot.motorLR.setPower(Range.clip((v3), -power, power));
            robot.motorRR.setPower(Range.clip((v4), -power, power));


            //
            /*  LIFT CONTROL  */
            if ((gamepad2.dpad_up)){
                targetPosition += 100;
                liftPower = 1;
            } else if ((gamepad2.dpad_down)) {
                targetPosition -= 100;
                drive.lowerClaw();
                liftPower = 1;
            } else if (gamepad2.dpad_left) {
                liftPos = 0;
                drive.resetBucket();
            } else if (gamepad2.dpad_right) {
                liftPos = 5000;
            }

            /* Limit the range of the lift so as not to damage the robot */
            targetPosition = Range.clip(targetPosition, robot.LIFT_RESET, robot.LIFT_MAX_HEIGHT);

            drive.liftPosition(targetPosition, liftPower);

            /* Claw Control */
            if (gamepad1.a && aRefresh == false){
                if(clawPos == true) {
                    drive.closeClaw();
                    clawPos = false;
//                    sleep(500);
                } else {
                    drive.openClaw();
                    clawPos = true;
//                    sleep(500);
                }

                aRefresh = true;
//                robot.launcherServo.setPosition(0);
            }

            if(!gamepad1.a) {
                if(aRefresh == true) {
                    aRefresh = false;
                }
            }

            if(gamepad2.left_stick_y > .35) {
                liftPos -= 100;
            } else if (gamepad2.left_stick_y < -.35) {
                liftPos += 100;
            }

            if(gamepad2.right_trigger > 0.1) {
                robot.bucketAxisServo.setPosition(.5);
            } else if(gamepad2.left_trigger > 0.1) {
                robot.bucketAxisServo.setPosition(0.01);
            }

            if(gamepad2.b && bRefresh == false) {
                if(clawAxisPos == 0) {
                    robot.launcherServo.setPosition(.4);
                    clawAxisPos = 1;
                    sleep(20);
                } else if(clawAxisPos == 1) {
                    robot.launcherServo.setPosition(0);
                    clawAxisPos = 0;
                    sleep(20);
                }

                bRefresh = true;
            }

            if(gamepad1.right_stick_button) {
                if(thetaMode == true) {
                    thetaPos = -90;
                    thetaPos = -90;

                    thetaMode = false;
                } else {
                    thetaPos = 90;
                    thetaPos =  90;

                    thetaMode = true;
                }
                sleep(500);
            }

            if(!gamepad2.b) bRefresh = false;

            if(gamepad2.right_bumper && passthroughMode == false) {
                elapsedTime.reset();
                passthroughMode = true;
                drive.closeClaw();
                sleep(200);
                robot.launcherServo.setPosition(.05);
            }

            if(gamepad2.left_bumper) {
                drive.launchDrone(true);
            } else {
                drive.launchDrone(false);
            }

            if(passthroughMode) {
                if(elapsedTime.time() > .5 && elapsedTime.time() < 1) {
                    robot.launcherServo.setPosition(1);
                    robot.bucketAxisServo.setPosition(.45);
//                    passthroughMode = false;
                } else if(elapsedTime.time() > 1.5 && elapsedTime.time() < 2) {
                    drive.openClaw();
//                    passthroughMode = false;
                } else if(elapsedTime.time() > 2.2 && elapsedTime.time() < 3) {
                    robot.launcherServo.setPosition(.4);
                    passthroughMode = false;
                }
            }

            liftPos = Range.clip(liftPos, 0, 1290);
            robot.slidesMotor.setTargetPosition((int) liftPos);
            robot.slidesMotor.setPower(1);

//            if(robot.sensorCone.getDistance(DistanceUnit.CM) <= 5) {
//                if(elapsedTime.time() >= 5) {
//                    drive.closeClaw();
//                }
//            }

            // Provide user feedback
            telemetry.addData("V1 = ", v1);
            telemetry.addData("elapsed time = ", elapsedTime.time());
            telemetry.addData("V2 = ", v2);
            telemetry.addData("V3 = ", v3);
            telemetry.addData("V4 = ", v4);
            telemetry.addData("Motor Left Lift = ", robot.motorLeftLift.getCurrentPosition());
            telemetry.addData("Target Position = ", targetPosition);
            telemetry.addData("Motor Right Lift = ", robot.motorRightLift.getCurrentPosition());
            telemetry.addData("Theta = ", theta);
            telemetry.addData("Lift pos = ", liftPos);
            telemetry.addData("Theta2 = ", theta);
            telemetry.addData("Claw Axis Value: ", clawAxisPos);
            telemetry.addData("theta pos:", thetaPos);
            telemetry.addData("theta mode:", thetaMode);

            telemetry.update();

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}       // end of MSTeleop class