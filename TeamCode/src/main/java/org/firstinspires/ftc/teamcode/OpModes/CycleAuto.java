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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.Hardware.RRHWProfile;
import org.firstinspires.ftc.teamcode.Libs.RRMechOps;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * FTC WIRES Autonomous Example for only vision detection using tensorflow and park
 */
//@Autonomous(name = "Auto - CTS", group = "Dev", preselectTeleOp = "Speedy TeleOp")
@Autonomous(name = "Cycle Auto", group = "Comp")
//@Disabled
public class CycleAuto extends LinearOpMode {

    public static String TEAM_NAME = "Mouse Spit";
    public static int TEAM_NUMBER = 11572;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    //Vision parameters
    private VisionOpenCV visionOpenCV;

    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    public static START_POSITION startPosition;

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

    @Override
    public void runOpMode() throws InterruptedException {

        //TODO: Initialize hardware
        robot.init(hardwareMap, false);

        int position = 3;

        //Key Pay inputs to selecting Starting Position of robot
        selectStartingPosition();

        //Activate Camera Vision that uses Open CV Vision processor for Team Element detection
        initOpenCV();

        // Wait for the DS start button to be touched.
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

        mechOps.clawleftclose();
        mechOps.clawRightClose();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Selected Starting Position", startPosition);

            //Run Open CV Object Detection and keep watching for the Team Element on the spike marks.
            runOpenCVObjectDetection();
        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {

            runAutonoumousMode();
        }
    }   // end runOpMode()

    public void runAutonoumousMode() {
        //Initialize Pose2d as desired
        Thread passthroughThread = null;
        Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
        Pose2d moveBeyondTrussPose = new Pose2d(0,0,0);
        Pose2d dropPurplePixelPose = new Pose2d(0, 0, 0);
        Pose2d midwayPose1 = new Pose2d(0,0,0);
        Pose2d midwayPose1a = new Pose2d(0,0,0);
        Pose2d intakeStack = new Pose2d(0,0,0);
        Pose2d moveToStack = new Pose2d(0,0,0);
        Pose2d midwayPose2 = new Pose2d(0,0,0);
        Pose2d moveForWhite = new Pose2d(0,0,0);
        Pose2d midwayPose3 = new Pose2d(0,0,0);
        Pose2d dropYellowPixelPose = new Pose2d(0, 0, 0);
        Pose2d parkPose = new Pose2d(0,0, 0);
        Pose2d parkPosePrep = new Pose2d(0,0, 0);
        double moveIntoBoard = 0;
        double waitSecondsBeforeDrop = 0;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        initPose = new Pose2d(0, 0, Math.toRadians(0)); //Starting pose
        moveBeyondTrussPose = new Pose2d(1,0,Math.toRadians(0));

        mechOps.clawleftclose();
        mechOps.clawRightClose();

        switch (startPosition) {
            case BLUE_LEFT:
                drive = new MecanumDrive(hardwareMap, initPose);
                switch(identifiedSpikeMarkLocation){
                    case LEFT:
//                        moveBeyondTrussPose = new Pose2d(33, 44, Math.toRadians(-90));
                        dropPurplePixelPose = new Pose2d(4, 11, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(21, 44, Math.toRadians(-90));
                        moveIntoBoard = 0;
                        break;
                    case MIDDLE:
                        moveBeyondTrussPose = new Pose2d(29, 28, Math.toRadians(-45));
                        dropPurplePixelPose = new Pose2d(21, 22, Math.toRadians(-50));
                        dropYellowPixelPose = new Pose2d(25, 42,  Math.toRadians(-90));
                        moveIntoBoard = 59;
                        break;
                    case RIGHT:
                        moveBeyondTrussPose = new Pose2d(33, 19, Math.toRadians(-90));
                        dropPurplePixelPose = new Pose2d(27, 14, Math.toRadians(-90));
                        dropYellowPixelPose = new Pose2d(28, 40, Math.toRadians(-90));
                        moveIntoBoard = 59;
                        break;
                }
                midwayPose1 = new Pose2d(14, 13, Math.toRadians(-45));
                waitSecondsBeforeDrop = 0; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(-2, 32, Math.toRadians(0));
                break;

            case RED_RIGHT:
                drive = new MecanumDrive(hardwareMap, initPose);
                switch(identifiedSpikeMarkLocation){
                    case LEFT:
                        moveBeyondTrussPose = new Pose2d(35, -16, Math.toRadians(90));
                        dropPurplePixelPose = new Pose2d(35, -7.5, Math.toRadians(90));
                        midwayPose1 = new Pose2d(42, -15, Math.toRadians(90));
                        dropYellowPixelPose = new Pose2d(38.5, -42, Math.toRadians(90));
                        break;
                    case MIDDLE:
                        moveBeyondTrussPose = new Pose2d(20, -18, Math.toRadians(90));
                        dropPurplePixelPose = new Pose2d(42, -24, Math.toRadians(90));
                        midwayPose1 = new Pose2d(42, -15, Math.toRadians(45));
                        dropYellowPixelPose = new Pose2d(31, -42,  Math.toRadians(90));
                        moveIntoBoard = 0;
                        break;
                    case RIGHT:
                        moveBeyondTrussPose = new Pose2d(30, -38, Math.toRadians(90));
                        dropPurplePixelPose = new Pose2d(35, -32, Math.toRadians(90));
                        midwayPose1 = new Pose2d(42, -15, Math.toRadians(45));
                        dropYellowPixelPose = new Pose2d(24, -42, Math.toRadians(90));
                        break;
                }
                parkPose = new Pose2d(2.5, -30, Math.toRadians(0));
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
                break;

            case BLUE_RIGHT:
                drive = new MecanumDrive(hardwareMap, initPose);
                switch(identifiedSpikeMarkLocation){
                    case LEFT:
//                        moveBeyondTrussPose = new Pose2d(40, -10, Math.toRadians(90));
                        dropPurplePixelPose = new Pose2d(13, -2, Math.toRadians(45));
                        dropYellowPixelPose = new Pose2d(17, 90, Math.toRadians(-90));
                        moveIntoBoard = 100;
                        midwayPose1a = new Pose2d(20, -10, Math.toRadians(36));
                        intakeStack = new Pose2d(53 , -1.5,Math.toRadians(-90));
                        break;
                    case MIDDLE:
                        moveBeyondTrussPose = new Pose2d(12, -5, Math.toRadians(0));
                        dropPurplePixelPose = new Pose2d(16, -2, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(25, 90, Math.toRadians(-90));
                        moveIntoBoard = 100;
                        midwayPose1a = new Pose2d(60, -15, Math.toRadians(-90));
                        intakeStack = new Pose2d(53, -2,Math.toRadians(-90));
//                        moveToStack = new Pose2d(65, -8, Math.toRadians(-90));
                        break;
                    case RIGHT:
                        moveBeyondTrussPose = new Pose2d(45, 0, Math.toRadians(-120));
                        dropPurplePixelPose = new Pose2d(55, 4, Math.toRadians(-160));
                        dropYellowPixelPose = new Pose2d(33.5, 90, Math.toRadians(-90));
                        moveIntoBoard = 100;
                        moveForWhite = new Pose2d(20, 90, Math.toRadians(-90));
                        midwayPose1a = new Pose2d(56, -10, Math.toRadians(-90));
                        intakeStack = new Pose2d(53, -1.5, Math.toRadians(-90));
                        break;
                }
                midwayPose2 = new Pose2d(50, 75, Math.toRadians(-90));
                waitSecondsBeforeDrop = 0; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(45, 95, Math.toRadians(0));
                break;

            case RED_LEFT:
                drive = new MecanumDrive(hardwareMap, initPose);
                switch(identifiedSpikeMarkLocation){
                    case LEFT:
                        moveBeyondTrussPose = new Pose2d(5, 0, Math.toRadians(0));
                        dropPurplePixelPose = new Pose2d(6, 3, Math.toRadians(25));
                        dropYellowPixelPose = new Pose2d(44, -92, Math.toRadians(90));
                        midwayPose1a = new Pose2d(60, 15, Math.toRadians(90));
                        intakeStack = new Pose2d(60, 23,Math.toRadians(90));
                        break;
                    case MIDDLE:
                        moveBeyondTrussPose = new Pose2d(10, 6, Math.toRadians(0));
                        dropPurplePixelPose = new Pose2d(20, 14, Math.toRadians(-40));
                        dropYellowPixelPose = new Pose2d(27, -90, Math.toRadians(90));
                        moveIntoBoard = -100;
                        midwayPose1a = new Pose2d(60, 22, Math.toRadians(90));
                        intakeStack = new Pose2d(52.6, 6.25,Math.toRadians(90));
                        break;
                    case RIGHT:
                        moveBeyondTrussPose = new Pose2d(10, 5, Math.toRadians(-32));
                        dropPurplePixelPose = new Pose2d(9, -2, Math.toRadians(-32));
                        dropYellowPixelPose = new Pose2d(21, -90, Math.toRadians(90));
                        moveForWhite = new Pose2d(40, -90, Math.toRadians(90));
                        moveIntoBoard = -100;
                        midwayPose1a = new Pose2d(18, 18, Math.toRadians(45));
                        intakeStack = new Pose2d(52.6, 6.9,Math.toRadians(90));
                        break;
                }
                midwayPose1 = new Pose2d(8, 8, Math.toRadians(0));
                midwayPose2 = new Pose2d(65, -66, Math.toRadians(90));
                waitSecondsBeforeDrop = 0; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(55, -84, Math.toRadians(0));
                break;
        }

        //Move robot to dropPurplePixel based on identified Spike Mark Location
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveBeyondTrussPose.position, moveBeyondTrussPose.heading)
                        .build());

        //Lower arm to push block out of the way
        mechOps.wristPosition(params.WRIST_EXTEND);
        mechOps.armLowIdle();
        sleep(500);

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(dropPurplePixelPose.position, dropPurplePixelPose.heading)
                        .build());

        //Lower arm to push block out of the way
//        mechOps.scoreLowPurplePixel();
        mechOps.armExtend();
        safeWaitSeconds(1.5);
        mechOps.clawLeftOpen();
        safeWaitSeconds(.5);
        mechOps.armIdleNoClose();
        safeWaitSeconds(.5);
        mechOps.clawleftclose();
        mechOps.slidesReset();
        mechOps.armReset();
//        safeWaitSeconds(1);
        if(startPosition == START_POSITION.RED_RIGHT && identifiedSpikeMarkLocation == IDENTIFIED_SPIKE_MARK_LOCATION.LEFT) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                            .build());
        }


        //Move robot to midwayPose1
//         Actions.runBlocking(
//         drive.actionBuilder(drive.pose)
//         .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
//         .build());

        //For Blue Right and Red Left, intake pixel from stack
        if (startPosition == START_POSITION.BLUE_RIGHT ||
                startPosition == START_POSITION.RED_LEFT) {
            if(moveToStack.position.x != 0 && moveToStack.position.y != 0) {
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(moveToStack.position, moveToStack.heading)
                                .build());
            } else {
//                opMode.sleep(30000); //debugging
            }

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose1a.position, midwayPose1a.heading)
                            .strafeToLinearHeading(intakeStack.position, intakeStack.heading)
                            .build());


//            if(startPosition == START_POSITION.BLUE_RIGHT) {
                mechOps.armExtendStack5();
                mechOps.clawLeftOpen();
                mechOps.wristPosition(0);
                safeWaitSeconds(1);
                mechOps.slidesExtend();
                safeWaitSeconds(1);
                mechOps.clawleftclose();
                safeWaitSeconds(1);
                mechOps.armIdle();
                safeWaitSeconds(1.5);
                passthroughThread = new Thread(() -> {
                    try {
                        mechOps.armIdle();
                        Thread.sleep(750);
                        mechOps.slidesReset();
                        Thread.sleep(1000);
                        mechOps.armReset();
                        mechOps.wristPosition(params.WRIST_LOAD_PIXELS);
                        Thread.sleep(1000);
                        mechOps.clawleftopenBucket();
                        Thread.sleep(1500);
                        mechOps.clawleftclose();
                        mechOps.armIdle();
                        mechOps.wristPosition(params.WRIST_EXTEND);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                });

                passthroughThread.start();
//                opMode.sleep(30000); //for testing
//            }

            //TODO : Code to intake pixel from stack
            safeWaitSeconds(0);

            //Move robot to midwayPose2
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
                            .build());
        }

//        safeWaitSeconds(waitSecondsBeforeDrop);
        //Move robot to midwayPose2 and to dropYellowPixelPose
        mechOps.armIdle();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
//                        .strafeToLinearHeading(midwayPose3.position, midwayPose3.heading)
                        .setReversed(true)
                        .splineToLinearHeading(dropYellowPixelPose,0)
//                        .lineToY(moveIntoBoard)
                        .build());

        mechOps.fingerHoldLeft();
        mechOps.fingerHoldRight();
        mechOps.bucketScore();

        while (passthroughThread.isAlive()) { // wait for thread to complete
            continue;
        }
        safeWaitSeconds(1 );
        //bandaid for rn
        if(moveIntoBoard != 0) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .lineToY(moveIntoBoard)
                            .build());
        }

        //TODO : Code to drop Pixel on Backdro p

//        mechOps.autoScore();
        mechOps.fingerReleaseRight();
        if(moveForWhite.position.x == 0 && moveForWhite.position.y == 0) {
            mechOps.fingerReleaseLeft();
        }
        mechOps.clawleftclose();
        mechOps.clawRightClose();
        safeWaitSeconds(1);

        if(moveForWhite.position.x != 0 && moveForWhite.position.y != 0) {
            mechOps.bucketScore();
            mechOps.fingerHoldLeft();
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(moveForWhite.position, moveForWhite.heading)
                            //.splineToLinearHeading(parkPose,0)
                            .build());
            mechOps.liftPosition(1000);
            safeWaitSeconds(.5);
            if(moveIntoBoard != 0) {
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .lineToY(moveIntoBoard)
                                .build());
            }
            mechOps.fingerReleaseRight();
            safeWaitSeconds(.5);
            mechOps.bucketReset();
            mechOps.liftPosition(0);
        }

        mechOps.slidesReset();
        mechOps.liftReset();
        mechOps.wristPosition(params.WRIST_LOAD_PIXELS);

        //Move robot to park in Backstage
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .lineToY(parkPose.position.y) //safety

                        .build());
        mechOps.bucketReset();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(parkPose.position, parkPose.heading)
                        //.splineToLinearHeading(parkPose,0)
                        .build());
        mechOps.armReset();
        safeWaitSeconds(10); //give time for park
    }


    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested()){
            telemetry.addData("Initializing FTC Wires (ftcwires.org) Autonomous adopted for Team:",
                    TEAM_NAME, " ", TEAM_NUMBER);
            telemetry.addData("---------------------------------------","");
            telemetry.addLine("This Auto program uses Open CV Vision Processor for Team Element detection");
            telemetry.addData("Select Starting Position using XYAB on Logitech (or ▢ΔOX on Playstayion) on gamepad 1:","");
            telemetry.addData("    Blue Left   ", "(X / ▢)");
            telemetry.addData("    Blue Right ", "(Y / Δ)");
            telemetry.addData("    Red Left    ", "(B / O)");
            telemetry.addData("    Red Right  ", "(A / X)");
            if(gamepad1.x){
                startPosition = START_POSITION.BLUE_LEFT;
                break;
            }
            if(gamepad1.y){
                startPosition = START_POSITION.BLUE_RIGHT;
                break;
            }
            if(gamepad1.b){
                startPosition = START_POSITION.RED_LEFT;
                break;
            }
            if(gamepad1.a){
                startPosition = START_POSITION.RED_RIGHT;
                break;
            }
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
            rectLeftOfCameraMid = new Rect(10, 200, 150, 140);
            rectRightOfCameraMid = new Rect(260, 250, 150, 200);
        } else { //RED_LEFT or BLUE_LEFT
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
            } else { //RED_RIGHT or BLUE_RIGHT
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