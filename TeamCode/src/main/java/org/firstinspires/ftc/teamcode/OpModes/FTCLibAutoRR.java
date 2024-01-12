package org.firstinspires.ftc.teamcode.OpModes;
/*
 */

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.RRHWProfile;
import org.firstinspires.ftc.teamcode.Libs.RRMechOps;
import org.firstinspires.ftc.teamcode.Hardware.Params;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Red Right Autonomous", group = "Concept")

public class FTCLibAutoRR extends LinearOpMode {

    FtcDashboard dashboard;
//    WebcamName camera = hardwareMap.get(WebcamName.class, "Webcam 1");

    /**
     VisionPortal visionPortal = VisionPortal.easyCreateWithDefaults(camera, visionProcessors);

     AprilTagProcessor myAprilTagProcessor;

     myAprilTagProcessor = new AprilTagProcessor.Builder()
     .setTagLibrary(myAprilTagLibrary)
     .setDrawTagID(true)
     .setDrawTagOutline(true)
     .setDrawAxes(true)
     .setDrawCubeProjection(true)
     .build();
     private static final String[] LABELS = {
     "circle",
     "star",
     "triangle"
     };
     **/

    private final static RRHWProfile robot = new RRHWProfile();
    private LinearOpMode opMode = this;
    private Params params = new Params();

    RRMechOps mechOps = new RRMechOps(robot, opMode, params);
    int position = 2;

    @Override

    public void runOpMode() {
        ElapsedTime elapsedTime = new ElapsedTime();

        /*
         * Setup the initial state of the robot
         */

//        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder().build();
//        aprilTag.setDecimation(2);
//        VisionPortal visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .addProcessor(aprilTag)
//                .build();
        State autoState = State.TEST;
        robot.init(hardwareMap, true);
        dashboard = FtcDashboard.getInstance();
        TelemetryPacket dashTelemetry = new TelemetryPacket();

        /* Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
//        mechOps.closeClaw();

        robot.parOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.parOdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.perpOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.perpOdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // detect position of init pixel
        while(!opModeIsActive() && !isStopRequested()) {


        }  // end of while

        while(opModeIsActive()){

            switch (autoState) {
                case TEST:
                    autoState = State.HALT;
                    mechOps.driveDistancePods(.2, 0, 16);
                    sleep(10);
                    mechOps.scorePurplePixel();
                    sleep(200);
                    mechOps.clawleftclose();
                    mechOps.clawRightClose();
                    mechOps.armIdle();
                    mechOps.driveDistancePods(.2, 270, 30);
                    sleep(10);
                    mechOps.PIDRotate(-90, 1);
                    sleep(10);
                    mechOps.driveDistancePods(.2, -135, 12);
                    sleep(10);
                    mechOps.bucketScore();
                    sleep(1000);
                    mechOps.driveDistancePods(.2, 0, 5);
                    sleep(10);
                    mechOps.bucketReset();
                    sleep(150);
                    mechOps.armReset();
                    mechOps.wristPosition(params.WRIST_LOAD_PIXELS);

                    break;

                case DETECT_POSITION:
                    telemetry.addData("PLACE POSITION = ", "TBD");
                    telemetry.update();

                    autoState = State.STATE_1;
                    break;

//                case STATE_1:
//                    drive.openClaw();
////                    sleep(150);
//                    drive.raiseClawToMid();
//                    sleep(125);
//                    drive.newDriveDistance(.5, 180, 21);
////                        sleep(10);
////                        drive.lowerClaw();
////                        sleep(150);
////                        drive.newDriveDistance(.4, 180, 2);
////                        drive.closeClaw();
////                        sleep(200);
////                        drive.newDriveDistance(.75, 0, 5);
////                        sleep(300);
////                        drive.raiseClawToMid();
//                    sleep(350);
//                    drive.PIDRotate(-90, 1);
//                    autoState = State.STATE_2;
//
//                    break;
//
//
//                case STATE_2:
//
//                    drive.liftPos(3750);
//                    drive.newDriveDistance(.5, 0, 35);
//                    drive.newDriveDistance(.25, 0, 4);
//                    sleep(400);
//                    drive.bucketScore();
////                    drive.bucketScore();
//                    sleep(1250);
//                    drive.resetBucket();
//                    sleep(150);
//                    drive.liftPos(0);
//
////                    if(elapsedTime.time() > 1 && elapsedTime.time() < 1.5) {
////                        drive.openClaw();
////                    } else if(elapsedTime.time() > 1.75) {
////                        robot.launcherServo.setPosition(.4);
////                    }
//                    autoState = State.STATE_3;
//                    break;
//                case STATE_3:
//
//                    if(position == 2) {
//                        drive.newDriveDistance(.5, 180, 40);
//                        drive.PIDRotate(0, 2);
//                        drive.lowerClaw();
//                        sleep(500);
//                        drive.newDriveDistance(.25, 180, 12);
//                        sleep(300);
//                        drive.closeClaw();
//                        sleep(500);
//                        drive.newDriveDistance(.25, 0, 9);
//                        drive.PIDRotate(-90, 2);
//                        drive.newDriveDistance(.4, 0, 39);
//                        drive.resetBucket();
//                        sleep(1250);
//                        robot.launcherServo.setPosition(.05);
//                        sleep(1250);
//                        drive.openClaw();
//                        sleep(500);
//                        drive.lowerClaw();
//                        drive.liftPos(3750);
//                        sleep(3000);
//                        drive.newDriveDistance(.25, 0,2);
//                        sleep(500 );
//                        drive.bucketScore();
//                        sleep(1250);
//                        drive.resetBucket();
//                        drive.liftPos(0);
//                        drive.newDriveDistance(.5, 180, 3);
//                        sleep(3500);
//                    }
//
//                    autoState = State.HALT;
//                    break;



                case PARK:

                    autoState = State.HALT;

                    break;

                case HALT:

                    // Stop all motors
                    sleep(1000);
//                    mechOps.();

                    // End the program
                    requestOpModeStop();

                    break;
            }   // end of the switch state


        } // end of while(opModeIsActive())
        // End the program
        requestOpModeStop();

    }

    enum State {
        TEST, DETECT_POSITION, STATE_1, STATE_2, STATE_3, PARK, HALT
    }   // end of enum State

    private void initAprilTag() {

    }
}