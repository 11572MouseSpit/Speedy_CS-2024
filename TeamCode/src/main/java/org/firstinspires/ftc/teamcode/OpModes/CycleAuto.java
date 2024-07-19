package org.firstinspires.ftc.teamcode.OpModes;
/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Hardware.RRHWProfile;
import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.Libs.RRMechOps;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.TimeUnit;

/**
 * FTC WIRES Autonomous Example for only vision detection using tensorflow and park
 */
//@Autonomous(name = "Auto - CTS", group = "Dev", preselectTeleOp = "Speedy TeleOp")
@Autonomous(name = "Cycle Auto", group = "Comp")
//@Disabled
public class CycleAuto extends LinearOpMode {

    public static String TEAM_NAME = "Mouse Spit";
    public static int TEAM_NUMBER = 11572;
    public static MecanumDrive drive;
    public static Thread readPositionThread;
    public static double autoDelay = 0;
    ElapsedTime autoTime = new ElapsedTime();

    public static final int RED_CENTER_SAFETY_DELAY = 5;
    public static final int RED_RIGHT_SAFETY_DELAY = 5;
    public static final int RED_LEFT_SAFETY_DELAY = 5;
    public static final int BLUE_CENTER_SAFETY_DELAY = 5;
    public static final int BLUE_RIGHT_SAFETY_DELAY = 5;
    public static final int BLUE_LEFT_SAFETY_DELAY = 5;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    public static Pose2d initPose;

    //Vision parameters
    private VisionOpenCV visionOpenCV;

    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_CENTER,
        BLUE_RIGHT,
        RED_LEFT,
        RED_CENTER,
        RED_RIGHT
    }
    public static START_POSITION startPosition;
    public static boolean dpadUpCooldown;
    public static boolean dpadDownCooldown;
    public static double headingOffset;
    public static int startPostionIndex = 0;
    public static boolean debug = false   ;

    public enum IDENTIFIED_SPIKE_MARK_LOCATION {
        LEFT,
        MIDDLE,
        RIGHT
    }
    public static IDENTIFIED_SPIKE_MARK_LOCATION identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
    public final static RRHWProfile robot = new RRHWProfile();
    public LinearOpMode opMode = this;
    public Params params = new Params();
    public RRMechOps mechOps = new RRMechOps(robot, opMode, params);

    public void blueLeftPark(Pose2d plannedPosition, MecanumDrive drive) {
        plannedPosition = new Pose2d(24, 30, Math.toRadians(-90) - headingOffset);
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                .build());
        plannedPosition = new Pose2d(0, 30, Math.toRadians(-90) - headingOffset);
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                .build());
        plannedPosition = new Pose2d(0, 50, Math.toRadians(-90) - headingOffset);
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .turnTo(plannedPosition.heading)
                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                .build());
        safeWaitSeconds(.25);
    }

    public void redRightPark(Pose2d plannedPosition, MecanumDrive drive) {
        plannedPosition = new Pose2d(24, -30, Math.toRadians(90) - headingOffset);
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                .build());
        plannedPosition = new Pose2d(0, -30, Math.toRadians(90) - headingOffset);
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                .build());
        plannedPosition = new Pose2d(0, -50, Math.toRadians(90) - headingOffset);
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                .build());
    }

    public void redCenterPark(Pose2d plannedPosition, MecanumDrive drive) {
        plannedPosition = new Pose2d(20, -30, Math.toRadians(90) - headingOffset);
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                .build());
        plannedPosition = new Pose2d(55, -30, Math.toRadians(90) - headingOffset);
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                .build());
        plannedPosition = new Pose2d(55, -35, Math.toRadians(90) - headingOffset);
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                .build());
    }

    @Override
    public void runOpMode() throws InterruptedException {

        //TODO: Initialize hardware
        robot.init(hardwareMap, false);



        initPose = new Pose2d(0, 0, 0); // Starting Pose

        drive = new MecanumDrive(hardwareMap, initPose);

        int position = 3;

        //Key Pay inputs to selecting Starting Position of robot
        selectStartingPosition();

        //Activate Camera Vision that uses Open CV Vision processor for Team Element detection
        initOpenCV();

        // Wait for the DS start button to be touched.
        telemetry.addData("Heading (can be -+ 5 deg): ", drive.pose.heading);
        telemetry.addData("Position (can be -+ .2 inch): ", drive.pose.position);
        telemetry.addLine();
        telemetry.addData("Selected Starting Position", startPosition);
        telemetry.addLine("Open CV Vision for Red/Blue Team Element Detection");
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addLine("The starting point of the robot is assumed to be on the starting tile, " +
                "and along the edge farther from the truss legs. ");
        telemetry.addLine("You should also have a webcam connected and positioned in a way to see " +
                "the middle spike mark and the spike mark away from the truss (and ideally nothing else). " +
                "We assumed the camera to be in the center of the robot. ");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        mechOps.clawLeftClose();
        mechOps.clawRightClose();
        mechOps.intakeReset();
        mechOps.droneLoad();
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setTargetPosition(0);
        robot.armMotor.setPower(.5);

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Delay: ", autoDelay);
            telemetry.addLine("Press dpad up/down to change the delay");
            telemetry.addLine();
            telemetry.addData("Heading (can be -+ 5 deg): ", drive.pose.heading);
            telemetry.addData("Position (can be -+ .2 inch): ", drive.pose.position);
            telemetry.addLine();
            telemetry.addData("Selected Starting Position", startPosition);

            if(gamepad1.dpad_up && !dpadUpCooldown) {
                dpadUpCooldown = true;
                autoDelay += 1;
            }

            if(!gamepad1.dpad_up) dpadUpCooldown = false;

            if(gamepad1.dpad_down && !dpadDownCooldown) {
                dpadDownCooldown = true;
                autoDelay -= 1;
                if(autoDelay < 0) autoDelay = 0;
            }

            if(!gamepad1.dpad_down) dpadDownCooldown = false;

            //Run Open CV Object Detection and keep watching for the Team Element on the spike marks.
            runOpenCVObjectDetection();
        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {

            runAutonoumousMode();
        }
    }   // end runOpMode()

    public void pauseUntilButton() {
        while (!gamepad1.a) {
            if(isStopRequested()) break;
        }
    }

    public void runAutonoumousMode() {
        autoTime.startTime();
        autoTime.reset();
        readPositionThread = new Thread(() -> {
            while (true) {
                if(isStopRequested()) break;
                telemetry.addData("x: ", drive.pose.position.x);
                telemetry.addData("y: ", drive.pose.position.y);
                telemetry.addData("right claw sense data:", robot.clawSensorRight.getNormalizedColors().blue);
                telemetry.addData("heading: ", Math.toDegrees(drive.pose.heading.toDouble()));
                telemetry.addData("corrected heading: ", Math.toDegrees(drive.pose.heading.toDouble() + headingOffset));
                telemetry.addData("90 deg corrected: ", Math.toDegrees(Math.toRadians(90) - headingOffset));
                telemetry.addData("heading offset: ", Math.toDegrees(headingOffset));
                telemetry.update();
                try {
                    Thread.sleep(200);
                } catch (InterruptedException e) {
                }
            }
        });

        readPositionThread.start();

        //Initialize Pose2d as desired
        Pose2d plannedPosition = new Pose2d(0, 0, 0); // Starting Pose
        Pose2d plannedPosition2 = new Pose2d(0, 0, 0); // Starting Pose
        Pose2d plannedPosition3 = new Pose2d(0, 0, 0); // Starting Pose
        Pose2d plannedPosition4 = new Pose2d(0, 0, 0); // Starting Pose
        Pose2d plannedPosition5 = new Pose2d(0, 0, 0); // Starting Pose

        drive.localizer.resetOdo();
        initPose = new Pose2d(0, 0, Math.toRadians(0)); //Starting pose
        drive.updatePoseEstimate();

//        new Thread(() -> {
//            while (true) {
//                    robot.climbMotorRight.setPower(-params.IDLE_CLIMB_DOWN);
//                    robot.climbMotorLeft.setPower(-params.IDLE_CLIMB_DOWN);
//                try {
//                    Thread.sleep(100);
//                } catch (InterruptedException e) {
//                    throw new RuntimeException(e);
//                }
//            }
//        }).start();

        switch (startPosition) {
            case BLUE_RIGHT:
                switch(identifiedSpikeMarkLocation){
                    case RIGHT:
                        safeWaitSeconds(autoDelay);

                        // drop pixel on line
                        mechOps.armCustomPos(params.ARM_SLIGHTLY_OFF_GROUND);
                        while (robot.armMotor.getCurrentPosition() < params.ARM_SLIGHTLY_OFF_GROUND - 50) continue;
                        plannedPosition = new Pose2d(15, 0, Math.toRadians(0));
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        plannedPosition = new Pose2d(30, -4, Math.toRadians(-50));
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        mechOps.armDownAuto();
                        while (robot.armMotor.getCurrentPosition() > 50) continue;
                        mechOps.clawRightOpenAuto();
                        mechOps.armCustomPos(params.ARM_ABOVE_BLOCK);
                        while (robot.armMotor.getCurrentPosition() < params.ARM_ABOVE_BLOCK - 50) continue;
                        plannedPosition = new Pose2d(5, 5, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());



                        // go to truss
//                        plannedPosition = new Pose2d(15, -4, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        plannedPosition = new Pose2d(15, 6, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        plannedPosition = new Pose2d(54.6, 6.4, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .turnTo(Math.toRadians(90))
//                                .build());
//                        safeWaitSeconds(.1);

//                        //move to left spot on board
//                        plannedPosition = new Pose2d(58.3, -50.17, Math.toRadians(90) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());

                        plannedPosition = new Pose2d(10, 120, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());

                        plannedPosition = new Pose2d(30, 120, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        //score
                        plannedPosition = new Pose2d(30, 141, Math.toRadians(-90) - headingOffset);
                        plannedPosition2 = new Pose2d(39, 141, Math.toRadians(-90) - headingOffset);
                        plannedPosition3 = new Pose2d(plannedPosition2.position.x, 135, Math.toRadians(-90) - headingOffset);
                        plannedPosition4 = new Pose2d(plannedPosition2.position.x, 140, Math.toRadians(-90) - headingOffset);
                        plannedPosition5 = new Pose2d(plannedPosition2.position.x, 144, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .strafeToLinearHeading(plannedPosition2.position, plannedPosition2.heading)
                                .strafeToLinearHeading(plannedPosition3.position, plannedPosition3.heading)

                                .build());
                        safeWaitSeconds(.1);
                        mechOps.clawRightClose();
                        mechOps.clawLeftClose();
                        mechOps.intakeRetract();
                        safeWaitSeconds(.1);
                        mechOps.armUpAuto();
                        mechOps.clawUpAuto();
                        mechOps.liftPositionNoReturn(params.LIFT_LOW_2_POSITION);
                        safeWaitSeconds(2);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition4.position, plannedPosition4.heading)
                                .strafeToLinearHeading(plannedPosition5.position, plannedPosition5.heading)

                                .build());
                        safeWaitSeconds(.2);
                        mechOps.clawLeftOpenAuto();
                        safeWaitSeconds(.2);
                        mechOps.clawDown();
                        mechOps.liftPositionNoReturn(0);
                        mechOps.armDownAuto();
                        safeWaitSeconds(1);
                        plannedPosition = new Pose2d(39, 128, Math.toRadians(-90) - headingOffset);
                        plannedPosition2 = new Pose2d(0, plannedPosition.position.y, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .turnTo(plannedPosition.heading)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .strafeToLinearHeading(plannedPosition2.position, plannedPosition2.heading)
                                .build());
                        while (autoTime.time(TimeUnit.SECONDS) < 23) continue;
                        plannedPosition = new Pose2d(30, plannedPosition2.position.y, Math.toRadians(0) - headingOffset);
                        plannedPosition2 = new Pose2d(30, 138, Math.toRadians(0) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .turnTo(plannedPosition.heading)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .strafeToLinearHeading(plannedPosition2.position, plannedPosition2.heading)
                                .build());
                        safeWaitSeconds(1);
                        safeWaitSeconds(.25);
                        break;
                    case MIDDLE:
                        // drop pixel on line
                        safeWaitSeconds(autoDelay);
                        if(debug) pauseUntilButton();
                        mechOps.armCustomPos(params.ARM_SLIGHTLY_OFF_GROUND);
                        while (robot.armMotor.getCurrentPosition() < params.ARM_SLIGHTLY_OFF_GROUND - 50) continue;
                        plannedPosition = new Pose2d(20, 7, Math.toRadians(0) - headingOffset);
                        plannedPosition2 = new Pose2d(31.5, 7, Math.toRadians(0) - headingOffset);
                        plannedPosition3 = new Pose2d(31.5, 0, Math.toRadians(0) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .strafeToLinearHeading(plannedPosition2.position, plannedPosition2.heading)
                                .strafeToLinearHeading(plannedPosition3.position, plannedPosition3.heading)
                                .build());
                        //mechOps.armDownAuto();

                        if(debug) pauseUntilButton();

                        safeWaitSeconds(.35);

//                        while (robot.armMotor.getCurrentPosition() > 50) continue;
                        mechOps.clawRightOpenAuto();
                        mechOps.armCustomPos(params.ARM_SLIGHTLY_OFF_GROUND);
                        safeWaitSeconds(.1);
                        plannedPosition = new Pose2d(15, 5, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());

                        if(debug) pauseUntilButton();


                        // go to truss
//                        plannedPosition = new Pose2d(15, -4, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        plannedPosition = new Pose2d(15, 6, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        plannedPosition = new Pose2d(54.6, 6.4, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .turnTo(Math.toRadians(90))
//                                .build());
//                        safeWaitSeconds(.1);

//                        //move to left spot on board
//                        plannedPosition = new Pose2d(58.3, -50.17, Math.toRadians(90) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());

                        plannedPosition = new Pose2d(15, 83.76, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());

                        if(debug) while (!gamepad1.a) continue;

                        plannedPosition = new Pose2d(30, 80, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        safeWaitSeconds(.1);
                        mechOps.clawRightClose();
                        mechOps.clawLeftClose();
                        mechOps.intakeRetract();
                        safeWaitSeconds(.1);
                        mechOps.armUpAuto();
                        mechOps.clawUpAuto();
                        mechOps.liftPositionNoReturn(params.LIFT_LOW_2_POSITION);
                        safeWaitSeconds(2);
                        //score
                        plannedPosition = new Pose2d(30, 86, Math.toRadians(-90) - headingOffset);
                        plannedPosition2 = new Pose2d(plannedPosition.position.x, 93.76, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .strafeToLinearHeading(plannedPosition2.position, plannedPosition2.heading)
                                .build());
                        safeWaitSeconds(.2);
                        mechOps.clawLeftOpenAuto();
                        safeWaitSeconds(.2);
                        plannedPosition = new Pose2d(50, 83.76, Math.toRadians(0) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        safeWaitSeconds(1);
                        mechOps.clawDown();
                        mechOps.liftPositionNoReturn(0);
                        mechOps.armDownAuto();
                        safeWaitSeconds(.25);
                        break;
                    case LEFT:
                        // drop pixel on line
                        safeWaitSeconds(autoDelay);

                        mechOps.armCustomPos(params.ARM_SLIGHTLY_OFF_GROUND);
                        while (robot.armMotor.getCurrentPosition() < params.ARM_SLIGHTLY_OFF_GROUND - 50) continue;
                        plannedPosition = new Pose2d(26, 15, Math.toRadians(0) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .splineTo(plannedPosition.position, plannedPosition.heading)
                                .build());
                        mechOps.armDownAuto();
                        while (robot.armMotor.getCurrentPosition() > 50) continue;
                        mechOps.clawRightOpenAuto();
                        mechOps.armCustomPos(params.ARM_ABOVE_BLOCK);
                        while (robot.armMotor.getCurrentPosition() < params.ARM_ABOVE_BLOCK - 50) continue;
                        plannedPosition = new Pose2d(10, 5, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());



                        // go to truss
//                        plannedPosition = new Pose2d(15, -4, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        plannedPosition = new Pose2d(15, 6, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        plannedPosition = new Pose2d(54.6, 6.4, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .turnTo(Math.toRadians(90))
//                                .build());
//                        safeWaitSeconds(.1);

//                        //move to left spot on board
//                        plannedPosition = new Pose2d(58.3, -50.17, Math.toRadians(90) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());

                        plannedPosition = new Pose2d(8, 83.76, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());

                        plannedPosition = new Pose2d(20, 83.76, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        safeWaitSeconds(.1);
                        mechOps.clawRightClose();
                        mechOps.clawLeftClose();
                        mechOps.intakeRetract();
                        safeWaitSeconds(.1);
                        mechOps.armUpAuto();
                        mechOps.clawUpAuto();
                        mechOps.liftPositionNoReturn(params.LIFT_LOW_2_POSITION);
                        safeWaitSeconds(1);
                        //score
                        plannedPosition = new Pose2d(25.5, 86, Math.toRadians(-90) - headingOffset);
                        plannedPosition2 = new Pose2d(plannedPosition.position.x, 93, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .strafeToLinearHeading(plannedPosition2.position, plannedPosition2.heading)
                                .build());
                        safeWaitSeconds(.2);
                        mechOps.clawLeftOpenAuto();
                        safeWaitSeconds(.2);
                        plannedPosition = new Pose2d(50, 83.76, Math.toRadians(0) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .turnTo(plannedPosition.heading)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        safeWaitSeconds(1);
                        mechOps.clawDown();
                        mechOps.liftPositionNoReturn(0);
                        mechOps.armDownAuto();
                        safeWaitSeconds(.25);
                        break;
                }
                break;
            case BLUE_CENTER:
                switch(identifiedSpikeMarkLocation){
                    case RIGHT:
                        // drop pixel on line
                        safeWaitSeconds(autoDelay);

                        mechOps.armCustomPos(params.ARM_SLIGHTLY_OFF_GROUND);
                        while (robot.armMotor.getCurrentPosition() < params.ARM_SLIGHTLY_OFF_GROUND - 50) continue;
                        plannedPosition = new Pose2d(20, 0, Math.toRadians(0));
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        plannedPosition = new Pose2d(25.58, -1.5, Math.toRadians(-50));
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        mechOps.armDownAuto();
                        while (robot.armMotor.getCurrentPosition() > 50) continue;
                        mechOps.clawRightOpenAuto();
                        mechOps.armCustomPos(params.ARM_ABOVE_BLOCK);
                        while (robot.armMotor.getCurrentPosition() < params.ARM_ABOVE_BLOCK - 50) continue;
                        plannedPosition = new Pose2d(10, 5, Math.toRadians(0) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());


                        // go to truss
//                        plannedPosition = new Pose2d(15, -4, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        plannedPosition = new Pose2d(15, 6, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        plannedPosition = new Pose2d(54.6, 6.4, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .turnTo(Math.toRadians(90))
//                                .build());
//                        safeWaitSeconds(.1);

//                        //move to left spot on board
//                        plannedPosition = new Pose2d(58.3, -50.17, Math.toRadians(90) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());

                        plannedPosition = new Pose2d(8, 80, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());

                        plannedPosition = new Pose2d(36, 80, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        safeWaitSeconds(.1);
                        mechOps.clawRightClose();
                        mechOps.clawLeftClose();
                        mechOps.intakeRetract();
                        safeWaitSeconds(.1);
                        mechOps.armUpAuto();
                        mechOps.clawUpAuto();
                        mechOps.liftPositionNoReturn(params.LIFT_LOW_2_POSITION);
                        safeWaitSeconds(1);
                        //score
                        plannedPosition = new Pose2d(36, 86, Math.toRadians(-90) - headingOffset);
                        plannedPosition2 = new Pose2d(36, 91.76, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .strafeToLinearHeading(plannedPosition2.position, plannedPosition2.heading)
                                .build());
                        safeWaitSeconds(.2);
                        mechOps.clawLeftOpenAuto();
                        safeWaitSeconds(.25);
                        mechOps.clawDown();
                        mechOps.liftPositionNoReturn(0);
                        mechOps.armDownAuto();
                        safeWaitSeconds(1);
                        plannedPosition = new Pose2d(36, 83.76, Math.toRadians(-90) - headingOffset);
                        plannedPosition2 = new Pose2d(57, 83.76, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .turnTo(plannedPosition.heading)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .strafeToLinearHeading(plannedPosition2.position, plannedPosition2.heading)
                                .build());
                        safeWaitSeconds(.25);
                        break;
                    case MIDDLE:
                        // drop pixel on line
                        safeWaitSeconds(autoDelay);

                        if(debug) pauseUntilButton();
                        mechOps.armCustomPos(params.ARM_SLIGHTLY_OFF_GROUND);
                        while (robot.armMotor.getCurrentPosition() < params.ARM_SLIGHTLY_OFF_GROUND - 50) continue;
                        plannedPosition = new Pose2d(20, 7, Math.toRadians(0) - headingOffset);
                        plannedPosition2 = new Pose2d(31.5, 7, Math.toRadians(0) - headingOffset);
                        plannedPosition3 = new Pose2d(31.5, 0, Math.toRadians(0) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .strafeToLinearHeading(plannedPosition2.position, plannedPosition2.heading)
                                .strafeToLinearHeading(plannedPosition3.position, plannedPosition3.heading)
                                .build());
                        //mechOps.armDownAuto();

                        if(debug) pauseUntilButton();

                        safeWaitSeconds(.35);

//                        while (robot.armMotor.getCurrentPosition() > 50) continue;
                        mechOps.clawRightOpenAuto();
                        mechOps.armCustomPos(params.ARM_SLIGHTLY_OFF_GROUND);
                        safeWaitSeconds(.1);
                        plannedPosition = new Pose2d(15, 5, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());

                        if(debug) pauseUntilButton();


                        // go to truss
//                        plannedPosition = new Pose2d(15, -4, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        plannedPosition = new Pose2d(15, 6, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        plannedPosition = new Pose2d(54.6, 6.4, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .turnTo(Math.toRadians(90))
//                                .build());
//                        safeWaitSeconds(.1);

//                        //move to left spot on board
//                        plannedPosition = new Pose2d(58.3, -50.17, Math.toRadians(90) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());

                        plannedPosition = new Pose2d(15, 75, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());

                        if(debug) while (!gamepad1.a) continue;

                        plannedPosition = new Pose2d(31.5, 75, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        safeWaitSeconds(.1);
                        mechOps.clawRightClose();
                        mechOps.clawLeftClose();
                        mechOps.intakeRetract();
                        safeWaitSeconds(.1);
                        mechOps.armUpAuto();
                        mechOps.clawUpAuto();
                        mechOps.liftPositionNoReturn(params.LIFT_LOW_2_POSITION);
                        safeWaitSeconds(2);
                        //score
                        plannedPosition = new Pose2d(31.5, 87, Math.toRadians(-90) - headingOffset);
                        plannedPosition2 = new Pose2d(plannedPosition.position.x, 93.76, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .strafeToLinearHeading(plannedPosition2.position, plannedPosition2.heading)
                                .build());
                        safeWaitSeconds(.25);
                        mechOps.clawLeftOpenAuto();

                        safeWaitSeconds(.25);
                        mechOps.clawDown();
                        mechOps.liftPositionNoReturn(0);
                        mechOps.armDownAuto();
                        safeWaitSeconds(.25);
                        plannedPosition = new Pose2d(36, 83.76, Math.toRadians(-90) - headingOffset);
                        plannedPosition2 = new Pose2d(57, 83.76, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .turnTo(plannedPosition.heading)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .strafeToLinearHeading(plannedPosition2.position, plannedPosition2.heading)
                                .build());
                        break;
                    case LEFT:
                        // drop pixel on line
                        safeWaitSeconds(autoDelay);

                        mechOps.armCustomPos(params.ARM_SLIGHTLY_OFF_GROUND);
                        while (robot.armMotor.getCurrentPosition() < params.ARM_SLIGHTLY_OFF_GROUND - 50) continue;
                        plannedPosition = new Pose2d(26, 17, Math.toRadians(0) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .splineTo(plannedPosition.position, plannedPosition.heading)
                                .build());
                        mechOps.armDownAuto();
                        while (robot.armMotor.getCurrentPosition() > 50) continue;
                        mechOps.clawRightOpenAuto();
                        mechOps.armCustomPos(params.ARM_ABOVE_BLOCK);
                        while (robot.armMotor.getCurrentPosition() < params.ARM_ABOVE_BLOCK - 50) continue;
                        plannedPosition = new Pose2d(10, 5, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());



                        // go to truss
//                        plannedPosition = new Pose2d(15, -4, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        plannedPosition = new Pose2d(15, 6, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        plannedPosition = new Pose2d(54.6, 6.4, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .turnTo(Math.toRadians(90))
//                                .build());
//                        safeWaitSeconds(.1);

//                        //move to left spot on board
//                        plannedPosition = new Pose2d(58.3, -50.17, Math.toRadians(90) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());

                        plannedPosition = new Pose2d(8, 83.76, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());

                        plannedPosition = new Pose2d(20, 83.76, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        safeWaitSeconds(.1);
                        mechOps.clawRightClose();
                        mechOps.clawLeftClose();
                        mechOps.intakeRetract();
                        safeWaitSeconds(.1);
                        mechOps.armUpAuto();
                        mechOps.clawUpAuto();
                        mechOps.liftPositionNoReturn(params.LIFT_LOW_2_POSITION);
                        safeWaitSeconds(1);
                        //score
                        plannedPosition = new Pose2d(25.5, 86, Math.toRadians(-90) - headingOffset);
                        plannedPosition2 = new Pose2d(plannedPosition.position.x, 93, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .strafeToLinearHeading(plannedPosition2.position, plannedPosition2.heading)
                                .build());
                        safeWaitSeconds(.2);
                        mechOps.clawLeftOpenAuto();
                        safeWaitSeconds(.2);
                        safeWaitSeconds(1);
                        mechOps.clawDown();
                        mechOps.liftPositionNoReturn(0);
                        mechOps.armDownAuto();
                        safeWaitSeconds(1);
                        plannedPosition = new Pose2d(36, 83.76, Math.toRadians(-90) - headingOffset);
                        plannedPosition2 = new Pose2d(57, 87, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .turnTo(plannedPosition.heading)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .strafeToLinearHeading(plannedPosition2.position, plannedPosition2.heading)
                                .build());
                        break;
                }
                break;
            case RED_CENTER:
                switch(identifiedSpikeMarkLocation){
                    case LEFT:
                        // drop pixel on line
                        safeWaitSeconds(autoDelay);

                        mechOps.armCustomPos(params.ARM_SLIGHTLY_OFF_GROUND);
                        while (robot.armMotor.getCurrentPosition() < params.ARM_SLIGHTLY_OFF_GROUND - 50) continue;
                        plannedPosition = new Pose2d(15, 0, Math.toRadians(0));
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        plannedPosition = new Pose2d(25.58, 9.32, Math.toRadians(50));
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        mechOps.armDownAuto();
                        while (robot.armMotor.getCurrentPosition() > 50) continue;
                        mechOps.clawRightOpenAuto();
                        mechOps.armCustomPos(params.ARM_ABOVE_BLOCK);
                        while (robot.armMotor.getCurrentPosition() < params.ARM_ABOVE_BLOCK - 50) continue;
                        plannedPosition = new Pose2d(10, -5, Math.toRadians(0) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());


                        // go to truss
//                        plannedPosition = new Pose2d(15, -4, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        plannedPosition = new Pose2d(15, 6, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        plannedPosition = new Pose2d(54.6, 6.4, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .turnTo(Math.toRadians(90))
//                                .build());
//                        safeWaitSeconds(.1);

//                        //move to left spot on board
//                        plannedPosition = new Pose2d(58.3, -50.17, Math.toRadians(90) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());

                        plannedPosition = new Pose2d(8, -80, Math.toRadians(90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());

                        plannedPosition = new Pose2d(33.5   , -80, Math.toRadians(90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        safeWaitSeconds(.1);
                        mechOps.clawRightClose();
                        mechOps.clawLeftClose();
                        mechOps.intakeRetract();
                        safeWaitSeconds(.1);
                        mechOps.armUpAuto();
                        mechOps.clawUpAuto();
                        mechOps.liftPositionNoReturn(params.LIFT_LOW_2_POSITION);
                        safeWaitSeconds(1);
                        //score
                        plannedPosition = new Pose2d(33.5, -91.76, Math.toRadians(90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        safeWaitSeconds(.2);
                        mechOps.clawLeftOpenAuto();
                        safeWaitSeconds(.2);
                        plannedPosition = new Pose2d(33.5, -83.76, Math.toRadians(0) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        safeWaitSeconds(.1);
                        mechOps.clawDown();
                        mechOps.liftPositionNoReturn(0);
                        mechOps.armDownAuto();
                        safeWaitSeconds(.25);
                        break;
                    case MIDDLE:
                        // drop pixel on line
                        safeWaitSeconds(autoDelay);

                        if(debug) pauseUntilButton();
                        mechOps.armCustomPos(params.ARM_ABOVE_BLOCK);
                        while (robot.armMotor.getCurrentPosition() < params.ARM_ABOVE_BLOCK - 50) continue;
                        plannedPosition = new Pose2d(31.5, 0, Math.toRadians(0) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        //mechOps.armDownAuto();

                        if(debug) pauseUntilButton();

                        safeWaitSeconds(.35);

//                        while (robot.armMotor.getCurrentPosition() > 50) continue;
                        mechOps.clawRightOpenAuto();
                        mechOps.armCustomPos(params.ARM_SLIGHTLY_OFF_GROUND);
                        safeWaitSeconds(.1);
                        plannedPosition = new Pose2d(15, -5, Math.toRadians(90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());

                        if(debug) pauseUntilButton();


                        // go to truss
//                        plannedPosition = new Pose2d(15, -4, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        plannedPosition = new Pose2d(15, 6, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        plannedPosition = new Pose2d(54.6, 6.4, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .turnTo(Math.toRadians(90))
//                                .build());
//                        safeWaitSeconds(.1);

//                        //move to left spot on board
//                        plannedPosition = new Pose2d(58.3, -50.17, Math.toRadians(90) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());

                        plannedPosition = new Pose2d(15, -83.76, Math.toRadians(90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());

                        if(debug) while (!gamepad1.a) continue;

                        plannedPosition = new Pose2d(26, -80, Math.toRadians(90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        safeWaitSeconds(.1);
                        mechOps.clawRightClose();
                        mechOps.clawLeftClose();
                        mechOps.intakeRetract();
                        safeWaitSeconds(.1);
                        mechOps.armUpAuto();
                        mechOps.clawUpAuto();
                        mechOps.liftPositionNoReturn(params.LIFT_LOW_2_POSITION);
                        safeWaitSeconds(2);
                        //score
                        plannedPosition = new Pose2d(26, -91.76, Math.toRadians(90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        safeWaitSeconds(.2);
                        mechOps.clawLeftOpenAuto();
                        safeWaitSeconds(.2);
                        plannedPosition = new Pose2d(40, -83.76, Math.toRadians(0) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        safeWaitSeconds(.1);
                        mechOps.clawDown();
                        mechOps.liftPositionNoReturn(0);
                        mechOps.armDownAuto();
                        safeWaitSeconds(.25);
                        break;
                    case RIGHT:
                        safeWaitSeconds(autoDelay);
                        // drop pixel on line
                        mechOps.armCustomPos(params.ARM_SLIGHTLY_OFF_GROUND);
                        while (robot.armMotor.getCurrentPosition() < params.ARM_SLIGHTLY_OFF_GROUND - 50) continue;
                        plannedPosition = new Pose2d(26, -7, Math.toRadians(0) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                         .splineTo(plannedPosition.position, plannedPosition.heading)
                         .build());
                          mechOps.armDownAuto();
                        while (robot.armMotor.getCurrentPosition() > 50) continue;
                        mechOps.clawRightOpenAuto();
                        mechOps.armCustomPos(params.ARM_ABOVE_BLOCK);
                        while (robot.armMotor.getCurrentPosition() < params.ARM_ABOVE_BLOCK - 50) continue;
                        plannedPosition = new Pose2d(10, -5, Math.toRadians(90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());



                        // go to truss
//                        plannedPosition = new Pose2d(15, -4, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        plannedPosition = new Pose2d(15, 6, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        plannedPosition = new Pose2d(54.6, 6.4, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .turnTo(Math.toRadians(90))
//                                .build());
//                        safeWaitSeconds(.1);

//                        //move to left spot on board
//                        plannedPosition = new Pose2d(58.3, -50.17, Math.toRadians(90) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());

                        plannedPosition = new Pose2d(8, -83.76, Math.toRadians(90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());

                        plannedPosition = new Pose2d(20, -83.76, Math.toRadians(90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        safeWaitSeconds(.1);
                        mechOps.clawRightClose();
                        mechOps.clawLeftClose();
                        mechOps.intakeRetract();
                        safeWaitSeconds(.1);
                        mechOps.armUpAuto();
                        mechOps.clawUpAuto();
                        mechOps.liftPositionNoReturn(params.LIFT_LOW_2_POSITION);
                        safeWaitSeconds(1);
                        //score
                        plannedPosition = new Pose2d(20, -91.76, Math.toRadians(90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        safeWaitSeconds(.2);
                        mechOps.clawLeftOpenAuto();
                        safeWaitSeconds(.2);
                        plannedPosition = new Pose2d(20, -83.76, Math.toRadians(0) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        safeWaitSeconds(.1);
                        mechOps.clawDown();
                        mechOps.liftPositionNoReturn(0);
                        mechOps.armDownAuto();
                        safeWaitSeconds(.25);
                        break;
                }
                break;
            case RED_RIGHT:
                switch(identifiedSpikeMarkLocation){
                    case LEFT:
                        safeWaitSeconds(autoDelay);
                        // drop pixel on line
                        mechOps.armCustomPos(params.ARM_SLIGHTLY_OFF_GROUND);
                        while (robot.armMotor.getCurrentPosition() < params.ARM_SLIGHTLY_OFF_GROUND - 50) continue;
                        plannedPosition = new Pose2d(15, 0, Math.toRadians(0));
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        plannedPosition = new Pose2d(25.58, 11.82, Math.toRadians(50));
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        mechOps.armDownAuto();
                        while (robot.armMotor.getCurrentPosition() > 50) continue;
                        mechOps.clawRightOpenAuto();
                        mechOps.armCustomPos(params.ARM_ABOVE_BLOCK);
                        while (robot.armMotor.getCurrentPosition() < params.ARM_ABOVE_BLOCK - 50) continue;
                        plannedPosition = new Pose2d(10, -5, Math.toRadians(90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());

                        // go to truss
//                        plannedPosition = new Pose2d(15, -4, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        plannedPosition = new Pose2d(15, 6, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        plannedPosition = new Pose2d(54.6, 6.4, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .turnTo(Math.toRadians(90))
//                                .build());
//                        safeWaitSeconds(.1);

//                        //move to left spot on board
//                        plannedPosition = new Pose2d(58.3, -50.17, Math.toRadians(90) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());

                        plannedPosition = new Pose2d(8, -30, Math.toRadians(90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());

                        plannedPosition = new Pose2d(33.5   , -30, Math.toRadians(90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        safeWaitSeconds(.1);
                        mechOps.clawRightClose();
                        mechOps.clawLeftClose();
                        mechOps.intakeRetract();
                        safeWaitSeconds(.1);
                        mechOps.armUpAuto();
                        mechOps.clawUpAuto();
                        mechOps.liftPositionNoReturn(params.LIFT_LOW_2_POSITION);
                        safeWaitSeconds(1);
                        //score
                        plannedPosition = new Pose2d(34.5, -42, Math.toRadians(90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        safeWaitSeconds(.5);
                        mechOps.clawLeftOpenAuto();
                        safeWaitSeconds(.2);
                        mechOps.clawDown();
                        mechOps.liftPositionNoReturn(0);
                        mechOps.armDownAuto();
                        safeWaitSeconds(.25);
                        redRightPark(plannedPosition, drive);
                        break;
                    case MIDDLE:
                        safeWaitSeconds(autoDelay);
                        // drop pixel on line
                        if(debug) pauseUntilButton();
                        mechOps.armCustomPos(params.ARM_SLIGHTLY_OFF_GROUND);
                        while (robot.armMotor.getCurrentPosition() < params.ARM_SLIGHTLY_OFF_GROUND - 50) continue;
                        plannedPosition = new Pose2d(15, -15, Math.toRadians(0) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        plannedPosition = new Pose2d(32, -15, Math.toRadians(0) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        plannedPosition = new Pose2d(32, 0, Math.toRadians(0) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        //mechOps.armDownAuto();

                        if(debug) pauseUntilButton();

                        safeWaitSeconds(.35);

//                        while (robot.armMotor.getCurrentPosition() > 50) continue;
                        mechOps.clawRightOpenAuto();
                        mechOps.armCustomPos(params.ARM_SLIGHTLY_OFF_GROUND);
                        safeWaitSeconds(.1);
                        plannedPosition = new Pose2d(15, -5, Math.toRadians(90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        if(debug) pauseUntilButton();


                        // go to truss
//                        plannedPosition = new Pose2d(15, -4, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        plannedPosition = new Pose2d(15, 6, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        plannedPosition = new Pose2d(54.6, 6.4, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .turnTo(Math.toRadians(90))
//                                .build());
//                        safeWaitSeconds(.1);

//                        //move to left spot on board
//                        plannedPosition = new Pose2d(58.3, -50.17, Math.toRadians(90) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());

                        plannedPosition = new Pose2d(15, -30, Math.toRadians(90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());

                        if(debug) while (!gamepad1.a) continue;

                        plannedPosition = new Pose2d(28, -30, Math.toRadians(90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        safeWaitSeconds(.1);
                        mechOps.clawRightClose();
                        mechOps.clawLeftClose();
                        mechOps.intakeRetract();
                        safeWaitSeconds(.1);
                        mechOps.armUpAuto();
                        mechOps.clawUpAuto();
                        mechOps.liftPositionNoReturn(params.LIFT_LOW_1_POSITION);
                        safeWaitSeconds(2);
                        //score
                        plannedPosition = new Pose2d(28, -42, Math.toRadians(90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        safeWaitSeconds(.2);
                        mechOps.clawLeftOpenAuto();
                        safeWaitSeconds(.1);
                        mechOps.clawDown();
                        mechOps.liftPositionNoReturn(0);
                        mechOps.armDownAuto();
                        safeWaitSeconds(1);
                        safeWaitSeconds(.2);
                        plannedPosition = new Pose2d(0, -42, Math.toRadians(90) - headingOffset);
                        plannedPosition2 = new Pose2d(0, -52, Math.toRadians(90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .strafeToLinearHeading(plannedPosition2.position, plannedPosition2.heading)
                                .build());
                        break;
                    case RIGHT:
                        safeWaitSeconds(autoDelay);
                        // drop pixel on line
                        mechOps.armCustomPos(params.ARM_SLIGHTLY_OFF_GROUND);
                        while (robot.armMotor.getCurrentPosition() < params.ARM_SLIGHTLY_OFF_GROUND - 50) continue;
                        plannedPosition = new Pose2d(26, -8 , Math.toRadians(0) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .splineTo(plannedPosition.position, plannedPosition.heading)
                                .build());
                        mechOps.armDownAuto();
                        while (robot.armMotor.getCurrentPosition() > 50) continue;
                        mechOps.clawRightOpenAuto();
                        mechOps.armCustomPos(params.ARM_ABOVE_BLOCK);
                        while (robot.armMotor.getCurrentPosition() < params.ARM_ABOVE_BLOCK - 50) continue;
                        plannedPosition = new Pose2d(15, -4.5, Math.toRadians(0) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        plannedPosition = new Pose2d(10, -4.5, Math.toRadians(90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());

                        // go to truss
//                        plannedPosition = new Pose2d(15, -4, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        plannedPosition = new Pose2d(15, 6, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        plannedPosition = new Pose2d(54.6, 6.4, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .turnTo(Math.toRadians(90))
//                                .build());
//                        safeWaitSeconds(.1);

//                        //move to left spot on board
//                        plannedPosition = new Pose2d(58.3, -50.17, Math.toRadians(90) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());

                        plannedPosition = new Pose2d(8, -30, Math.toRadians(90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());

                        plannedPosition = new Pose2d(20, -30, Math.toRadians(90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        safeWaitSeconds(.1);
                        mechOps.clawRightClose();
                        mechOps.clawLeftClose();
                        mechOps.intakeRetract();
                        safeWaitSeconds(.1);
                        mechOps.armUpAuto();
                        mechOps.clawUpAuto();
                        mechOps.liftPositionNoReturn(params.LIFT_LOW_2_POSITION);
                        safeWaitSeconds(1);
                        //score
                        plannedPosition = new Pose2d(20, -42, Math.toRadians(90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        safeWaitSeconds(.2);
                        mechOps.clawLeftOpenAuto();
                        safeWaitSeconds(.25);
                        mechOps.clawDown();
                        mechOps.liftPositionNoReturn(0);
                        mechOps.armDownAuto();
                        safeWaitSeconds(1);
                        redCenterPark(plannedPosition, drive);

                        break;
                }
                break;
            case BLUE_LEFT:
                switch(identifiedSpikeMarkLocation) {
                    case LEFT:
                        safeWaitSeconds(autoDelay);
                        // drop pixel on line
                        mechOps.armCustomPos(params.ARM_SLIGHTLY_OFF_GROUND);
                        while (robot.armMotor.getCurrentPosition() < params.ARM_SLIGHTLY_OFF_GROUND - 50) continue;
                        plannedPosition = new Pose2d(15, 0, Math.toRadians(0));
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        plannedPosition = new Pose2d(25.58, 7.8, Math.toRadians(50));
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        mechOps.armDownAuto();
                        while (robot.armMotor.getCurrentPosition() > 50) continue;
                        mechOps.clawRightOpenAuto();
                        mechOps.armCustomPos(params.ARM_ABOVE_BLOCK);
                        while (robot.armMotor.getCurrentPosition() < params.ARM_ABOVE_BLOCK - 50) continue;
                        plannedPosition = new Pose2d(10, 5, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());

                        // go to truss
//                        plannedPosition = new Pose2d(15, -4, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        plannedPosition = new Pose2d(15, 6, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        plannedPosition = new Pose2d(54.6, 6.4, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .turnTo(Math.toRadians(90))
//                                .build());
//                        safeWaitSeconds(.1);

//                        //move to left spot on board
//                        plannedPosition = new Pose2d(58.3, -50.17, Math.toRadians(90) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());

                        plannedPosition = new Pose2d(8, 30, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());

                        plannedPosition = new Pose2d(24, 30, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        safeWaitSeconds(.1);
                        mechOps.clawRightClose();
                        mechOps.clawLeftClose();
                        mechOps.intakeRetract();
                        safeWaitSeconds(.1);
                        mechOps.armUpAuto();
                        mechOps.clawUpAuto();
                        mechOps.liftPositionNoReturn(params.LIFT_LOW_1_POSITION);
                        safeWaitSeconds(1);
                        //score
                        plannedPosition = new Pose2d(24, 43, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        safeWaitSeconds(.25);
                        mechOps.clawLeftOpenAuto();
                        safeWaitSeconds(.25);
                        mechOps.clawDown();
                        mechOps.liftPositionNoReturn(0);
                        mechOps.armDownAuto();
                        safeWaitSeconds(.5);
//                        mechOps.clawDown();
                        blueLeftPark(plannedPosition, drive);
                        safeWaitSeconds(.25);
                        break;
                    case MIDDLE:
                        // drop pixel on
                        safeWaitSeconds(autoDelay);

                        if(debug) pauseUntilButton();
                        mechOps.armCustomPos(params.ARM_SLIGHTLY_OFF_GROUND);
                        while (robot.armMotor.getCurrentPosition() < params.ARM_SLIGHTLY_OFF_GROUND - 50) continue;
                        plannedPosition = new Pose2d(15, 15, Math.toRadians(0) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        plannedPosition = new Pose2d(32, 15, Math.toRadians(0) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        plannedPosition = new Pose2d(32, 5, Math.toRadians(0) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        //mechOps.armDownAuto();

                        if(debug) pauseUntilButton();


//                        while (robot.armMotor.getCurrentPosition() > 50) continue;
                        sleep(350);
                        mechOps.clawRightOpenAuto();
                        sleep(500);
                        safeWaitSeconds(.1);
//                        plannedPosition = new Pose2d(8, 15, Math.toRadians(-90) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        safeWaitSeconds(.1);
//                        plannedPosition = new Pose2d(15, 15, Math.toRadians(-90) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        safeWaitSeconds(.1);
//                        if(debug) pauseUntilButton();


                        // go to truss
//                        plannedPosition = new Pose2d(15, -4, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        plannedPosition = new Pose2d(15, 6, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        plannedPosition = new Pose2d(54.6, 6.4, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .turnTo(Math.toRadians(90))
//                                .build());
//                        safeWaitSeconds(.1);

//                        //move to left spot on board
//                        plannedPosition = new Pose2d(58.3, -50.17, Math.toRadians(90) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());

//                        plannedPosition = new Pose2d(30, 30, Math.toRadians(-90) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());

                        if(debug) while (!gamepad1.a) continue;

                        plannedPosition = new Pose2d(30, 30, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        safeWaitSeconds(.1);
                        mechOps.clawRightClose();
                        mechOps.clawLeftClose();
                        mechOps.intakeRetract();
                        safeWaitSeconds(.25);
                        mechOps.armUpAuto();
                        mechOps.clawUpAuto();
                        mechOps.liftPositionNoReturn(params.LIFT_LOW_1_POSITION);
                        safeWaitSeconds(2);
                        //score
                        plannedPosition = new Pose2d(32.5, 42, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        safeWaitSeconds(.2);
                        mechOps.clawLeftOpenAuto();
                        safeWaitSeconds(.2);
                        plannedPosition = new Pose2d(30, 35, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        safeWaitSeconds(.5);
                        mechOps.clawDown();
                        mechOps.liftPositionNoReturn(0);
                        mechOps.armDownAuto();
                        safeWaitSeconds(.25);
                        blueLeftPark(plannedPosition, drive);

                        break;
                    case RIGHT:
                        // drop pixel on line
                        safeWaitSeconds(autoDelay);
                        mechOps.armCustomPos(params.ARM_SLIGHTLY_OFF_GROUND);
                        while (robot.armMotor.getCurrentPosition() < params.ARM_SLIGHTLY_OFF_GROUND - 50) continue;
                        plannedPosition = new Pose2d(15, 0, Math.toRadians(0));
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        plannedPosition = new Pose2d(30, -7, Math.toRadians(-50));
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        mechOps.armDownAuto();
                        while (robot.armMotor.getCurrentPosition() > 50) continue;
                        mechOps.clawRightOpenAuto();
                        mechOps.armCustomPos(params.ARM_ABOVE_BLOCK);
                        while (robot.armMotor.getCurrentPosition() < params.ARM_ABOVE_BLOCK - 50) continue;
                        plannedPosition = new Pose2d(10, 5, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());

                        // go to truss
//                        plannedPosition = new Pose2d(15, -4, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        plannedPosition = new Pose2d(15, 6, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        plannedPosition = new Pose2d(54.6, 6.4, Math.toRadians(0) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .turnTo(Math.toRadians(90))
//                                .build());
//                        safeWaitSeconds(.1);

//                        //move to left spot on board
//                        plannedPosition = new Pose2d(58.3, -50.17, Math.toRadians(90) - headingOffset);
//                        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
//                                .build());

                        plannedPosition = new Pose2d(8, 30, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());

                        plannedPosition = new Pose2d(33.5, 30, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .build());
                        safeWaitSeconds(.1);
                        mechOps.clawRightClose();
                        mechOps.clawLeftClose();
                        mechOps.intakeRetract();
                        safeWaitSeconds(.1);
                        mechOps.armUpAuto();
                        mechOps.clawUpAuto();
//                        mechOps.liftPositionNoReturn(params.LIFT_LOW_2_POSITION);
                        safeWaitSeconds(1);
                        //score
                        plannedPosition = new Pose2d(36, 38, Math.toRadians(-90) - headingOffset);
                        plannedPosition2 = new Pose2d(36, 42.8, Math.toRadians(-90) - headingOffset);
                        Actions.runBlocking(drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(plannedPosition.position, plannedPosition.heading)
                                .strafeToLinearHeading(plannedPosition2.position, plannedPosition2.heading)
                                .build());
                        safeWaitSeconds(1);
                        mechOps.clawLeftOpenAuto();
                        safeWaitSeconds(.25);
                        mechOps.clawDown();
                        mechOps.liftPositionNoReturn(0);
                        mechOps.armDownAuto();
                        safeWaitSeconds(.25);
                        blueLeftPark(plannedPosition, drive);
                        break;
                }
        }
        while (robot.armMotor.getCurrentPosition() > 10) continue;
        safeWaitSeconds(.5); //give time for park
    }


    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested()){
            telemetry.addLine("DPad up/down to cycle through positions");
            telemetry.addLine("A to select position");
            startPosition = START_POSITION.values()[startPostionIndex];
            telemetry.addData("Selected Position: ", startPosition.name());

            if(gamepad1.dpad_up && !dpadUpCooldown) {
                dpadUpCooldown = true;

                startPostionIndex++;

                if(startPostionIndex > (START_POSITION.values().length - 1)) startPostionIndex = 0;
            }

            if(!gamepad1.dpad_up) dpadUpCooldown = false;

            if(gamepad1.dpad_down && !dpadDownCooldown) {
                dpadDownCooldown = true;

                startPostionIndex--;

                if(startPostionIndex < 0) startPostionIndex = START_POSITION.values().length - 1;
            }

            if(!gamepad1.dpad_down) dpadDownCooldown = false;

            if(gamepad1.a) break;

            telemetry.update();
        }
        telemetry.clearAll();
    }

    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }

    /**
     * Initialize the Open CV Object Detection processor.
     */
    public Rect rectLeftOfCameraMid, rectRightOfCameraMid;
    private void initOpenCV() {
        visionOpenCV = new VisionOpenCV(hardwareMap);

        if (startPosition == START_POSITION.RED_RIGHT ||
                startPosition == START_POSITION.BLUE_RIGHT) {
            rectLeftOfCameraMid = new Rect(10, 200, 275, 125);
            rectRightOfCameraMid = new Rect(400, 200, 150, 240);
        } else if (startPosition == START_POSITION.RED_LEFT ||
                startPosition == START_POSITION.BLUE_LEFT) { //RED_LEFT or BLUE_LEFT
            rectLeftOfCameraMid = new Rect(10, 200, 275, 125);
            rectRightOfCameraMid = new Rect(400, 200, 150, 240);
        } else if (startPosition == START_POSITION.RED_CENTER ||
                startPosition == START_POSITION.BLUE_CENTER) { //RED_LEFT or BLUE_LEFT
            rectLeftOfCameraMid = new Rect(10, 200, 275, 125);
            rectRightOfCameraMid = new Rect(400, 200, 150, 240);
        }
    }

    /**
     * Add telemetry about Object Detection recognitions.
     */
    private void runOpenCVObjectDetection() {
        visionOpenCV.getSelection();
        telemetry.addLine("Open CV based Vision Processor for Team Element Detection");
        telemetry.addData("Identified Parking Location", identifiedSpikeMarkLocation);
        telemetry.addData("SatLeftOfCameraMid", visionOpenCV.satRectLeftOfCameraMid);

        telemetry.addData("SatRightOfCameraMid", visionOpenCV.satRectRightOfCameraMid);
        telemetry.addData("SatRectNone", visionOpenCV.satRectNone);
        telemetry.update();
    }

    public class VisionOpenCV implements VisionProcessor {

        CameraSelectedAroundMid selectionAroundMid = CameraSelectedAroundMid.NONE;

        public VisionPortal visionPortal;

        Mat submat = new Mat();
        Mat hsvMat = new Mat();

        public double satRectLeftOfCameraMid, satRectRightOfCameraMid;
        public double satRectNone = 40.0;

        public VisionOpenCV(HardwareMap hardwareMap){
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), this);
        }

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

            satRectLeftOfCameraMid = getAvgSaturation(hsvMat, rectLeftOfCameraMid);
            satRectRightOfCameraMid = getAvgSaturation(hsvMat, rectRightOfCameraMid);

            if ((satRectLeftOfCameraMid > satRectRightOfCameraMid) && (satRectLeftOfCameraMid > satRectNone)) {
                return CameraSelectedAroundMid.LEFT_OF_CAMERA_MID;
            } else if ((satRectRightOfCameraMid > satRectLeftOfCameraMid) && (satRectRightOfCameraMid > satRectNone)) {
                return CameraSelectedAroundMid.RIGHT_OF_CAMERA_MID;
            }
            return CameraSelectedAroundMid.NONE;
        }

        protected double getAvgSaturation(Mat input, Rect rect) {
            submat = input.submat(rect);
            Scalar color = Core.mean(submat);
            return color.val[1];
        }

        private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
            int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
            int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
            int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
            int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

            return new android.graphics.Rect(left, top, right, bottom);
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            Paint selectedPaint = new Paint();
            selectedPaint.setColor(Color.RED);
            selectedPaint.setStyle(Paint.Style.STROKE);
            selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

            Paint nonSelectedPaint = new Paint(selectedPaint);
            nonSelectedPaint.setColor(Color.GREEN);

            android.graphics.Rect drawRectangleLeft = makeGraphicsRect(rectLeftOfCameraMid, scaleBmpPxToCanvasPx);
            android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectRightOfCameraMid, scaleBmpPxToCanvasPx);

            selectionAroundMid = (CameraSelectedAroundMid) userContext;
            switch (selectionAroundMid) {
                case LEFT_OF_CAMERA_MID:
                    canvas.drawRect(drawRectangleLeft, selectedPaint);
                    canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                    break;
                case RIGHT_OF_CAMERA_MID:
                    canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                    canvas.drawRect(drawRectangleMiddle, selectedPaint);
                    break;
                case NONE:
                    canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                    canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                    break;
            }
        }

        public void getSelection() {
            if (startPosition == START_POSITION.RED_LEFT ||
                    startPosition == START_POSITION.BLUE_LEFT) {
                switch (selectionAroundMid) {
                    case NONE:
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
                        break;
                    case LEFT_OF_CAMERA_MID:
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
                        break;
                    case RIGHT_OF_CAMERA_MID:
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;
                        break;
                }
            } else if(startPosition == START_POSITION.RED_RIGHT || startPosition == START_POSITION.BLUE_RIGHT) { //RED_RIGHT or BLUE_RIGHT
                switch (selectionAroundMid) {
                    case NONE:
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
                        break;
                    case LEFT_OF_CAMERA_MID:
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
                        break;
                    case RIGHT_OF_CAMERA_MID:
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;
                        break;
                }
            } else {
                switch (selectionAroundMid) {
                    case NONE:
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
                        break;
                    case LEFT_OF_CAMERA_MID:
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
                        break;
                    case RIGHT_OF_CAMERA_MID:
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;
                        break;
                }
            }
        }
    }

    public enum CameraSelectedAroundMid {
        NONE,
        LEFT_OF_CAMERA_MID,
        RIGHT_OF_CAMERA_MID
    }

}   // end class