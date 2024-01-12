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
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 * <p>
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Red Right Autonomous", group = "Concept")

public class FTCLibAutoRR extends LinearOpMode {

    FtcDashboard dashboard;
//    WebcamName camera = hardwareMap.get(WebcamName.class, "Webcam 1");

    /**
     * VisionPortal visionPortal = VisionPortal.easyCreateWithDefaults(camera, visionProcessors);
     * <p>
     * AprilTagProcessor myAprilTagProcessor;
     * <p>
     * myAprilTagProcessor = new AprilTagProcessor.Builder()
     * .setTagLibrary(myAprilTagLibrary)
     * .setDrawTagID(true)
     * .setDrawTagOutline(true)
     * .setDrawAxes(true)
     * .setDrawCubeProjection(true)
     * .build();
     * private static final String[] LABELS = {
     * "circle",
     * "star",
     * "triangle"
     * };
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
        State autoState = State.LEFT;
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
        while (!opModeIsActive() && !isStopRequested()) {


        }  // end of while

        while (opModeIsActive()) {
            elapsedTime.reset();
            double parStart = robot.parOdo.getCurrentPosition();
            double perpStart = robot.perpOdo.getCurrentPosition();

            switch (autoState) {
                case CENTER:
                    mechOps.clawleftclose();
                    mechOps.clawRightClose();
                    sleep(500);
                    mechOps.armLowIdle();
                    mechOps.wristPosition(params.WRIST_RESET);
                    sleep(1250);
                    mechOps.driveDistancePods(.2, 0, 17.5);
                    telemetry.addData("distanceTravelled", mechOps.calcOdoDistance(parStart, perpStart));
                    telemetry.addData("x+y travel", "par: " + mechOps.calcParOdoDistance(parStart) + " perp: " + mechOps.calcPerpOdoDistance(perpStart));
                    telemetry.update();
//                    mechOps.driveDistancePods(.2, 270, 4);
                    parStart = robot.parOdo.getCurrentPosition();
                    perpStart = robot.perpOdo.getCurrentPosition();
                    telemetry.addData("distanceTravelled", mechOps.calcOdoDistance(parStart, perpStart));
//                    telemetry.update();
                    mechOps.armIdle();
                    sleep(1000);
                    mechOps.wristPosition(params.WRIST_LOAD_PIXELS);
                    sleep(1000);
                    mechOps.scorePurplePixel();
                    mechOps.driveDistancePods(.2, 90, 4);
                    sleep(200);
                    mechOps.clawleftclose();
                    mechOps.clawRightClose();
                    mechOps.armIdle();
                    mechOps.driveDistancePods(.2, 90, 20);
                    sleep(10);
                    mechOps.PIDRotate(90, 1);
                    parStart = robot.parOdo.getCurrentPosition();
                    perpStart = robot.perpOdo.getCurrentPosition();
                    mechOps.driveDistancePods(.2, 90, 3);
                    telemetry.addData("distanceTravelled", mechOps.calcOdoDistance(parStart, perpStart));
//                    telemetry.update();
                    sleep(100);
                    mechOps.driveDistance(.2, 0, 10);
                    sleep(20);
                    mechOps.bucketScore();
                    sleep(1000);
                    break;
                case RIGHT:
                    mechOps.slidesReset();
                    sleep(200);
                    mechOps.clawleftclose();
                    mechOps.clawRightClose();
                    mechOps.armIdle();
                    sleep(100);
                    mechOps.driveDistancePods(.35, 0, 18);
                    sleep(20); //delay to check error
                    mechOps.driveDistancePods(.35, 90, 25);
                    sleep(20);
                    mechOps.PIDRotate(90, 1);
                    mechOps.driveDistancePods(.35, 90, 1.5);
                    sleep(20);
                    mechOps.driveDistance(.3, 0, 15);
                    sleep(500);
                    mechOps.bucketScore();
                    sleep(1250);
                    mechOps.bucketReset();
                    sleep(200);
                    mechOps.driveDistancePods(.2, 0, 3.5);
                    sleep(20);
                    mechOps.driveDistancePods(.2, 90,13.5);
                    sleep(20);
                    mechOps.armLowIdle();
                    mechOps.wristPosition(params.WRIST_EXTEND);
                    sleep(1000);
                    mechOps.driveDistancePods(.2, 0,7);
                    sleep(20);
                    mechOps.driveDistancePods(.2, 180,1);
                    sleep(20);
//                    mechOps.armIdle();
                    sleep(500);
//                    mechOps.scorePurplePixel();
                    mechOps.clawLeftOpen();
                    sleep(1000);
                    mechOps.driveDistance(.1, 0, 6);
                    mechOps.clawleftclose();
                    mechOps.clawRightClose();
                    sleep(500);
                    mechOps.armReset();
                    mechOps.wristPosition(params.WRIST_LOAD_PIXELS);
                    break;

                case LEFT:
                    mechOps.slidesReset();
                    sleep(200);
                    mechOps.clawleftclose();
                    mechOps.clawRightClose();
                    mechOps.armIdle();
                    sleep(100);
                    mechOps.driveDistancePods(.35, 0, 18);
                    sleep(500); //delay to check error
                    mechOps.driveDistancePods(.35, 90, 25);
                    sleep(20);
                    mechOps.PIDRotate(90, 1);
                    mechOps.driveDistancePods(.35, 90, 1.5);
                    sleep(20);
                    mechOps.driveDistance(.3, 0, 15);
                    sleep(500);
                    mechOps.bucketScore();
                    sleep(1250);
                    mechOps.bucketReset();
                    sleep(200);
                    mechOps.driveDistancePods(.2, 0, 3.5);
                    sleep(20);
                    mechOps.driveDistancePods(.2, 90,13.5);
                    sleep(20);
                    mechOps.armLowIdle();
                    mechOps.wristPosition(params.WRIST_EXTEND);
                    sleep(1000);
                    mechOps.driveDistancePods(.2, 0,27);
                    sleep(20);
//                    mechOps.driveDistancePods(.2, 180,1);
//                    sleep(20);
//                    mechOps.armIdle();
                    sleep(500);
//                    mechOps.scorePurplePixel();
                    mechOps.clawLeftOpen();
                    sleep(1000);
                    mechOps.driveDistancePods(.2, 180, 27);
                    mechOps.clawleftclose();
                    mechOps.clawRightClose();
                    sleep(500);
                    mechOps.armReset();
                    mechOps.wristPosition(params.WRIST_LOAD_PIXELS);
                    break;

            }
//            mechOps.driveDistance(.2, 315, 18);

            mechOps.driveDistancePods(.2, 0, 5);
            sleep(10);
            mechOps.bucketReset();
            sleep(150);
            mechOps.armReset();
            mechOps.wristPosition(params.WRIST_LOAD_PIXELS);

            sleep((long) ((30 - elapsedTime.time()) * 1000));

            break; //kill loop
        } // end of while(opModeIsActive())
        // End the program
        requestOpModeStop();

    }

    enum State {
        TEST, DETECT_POSITION, STATE_1, STATE_2, STATE_3, PARK, CENTER, RIGHT, LEFT, HALT
    }   // end of enum State

    private void initAprilTag() {

    }
}