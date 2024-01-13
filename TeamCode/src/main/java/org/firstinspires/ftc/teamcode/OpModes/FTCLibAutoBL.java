package org.firstinspires.ftc.teamcode.OpModes;
/*
 */

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Hardware.RRHWProfile;
import org.firstinspires.ftc.teamcode.Libs.RRMechOps;
import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

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

public class FTCLibAutoBL extends LinearOpMode {

    FtcDashboard dashboard;
    RRTestCode.START_POSITION startPosition;
    static State autoState = State.LEFT;


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
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    //Vision parameters
    private RRTestCode.VisionOpenCV visionOpenCV;
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
        robot.init(hardwareMap, true);
        dashboard = FtcDashboard.getInstance();
        TelemetryPacket dashTelemetry = new TelemetryPacket();

        /* Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
//        mechOps.closeClaw();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new SamplePipeline();
        webcam.setPipeline(pipeline);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        robot.parOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.parOdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.perpOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.perpOdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // detect position of init pixel
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Type", pipeline.getType());
            telemetry.addData("Average", pipeline.getAverage());
            telemetry.addData("average1", pipeline.average1);
            telemetry.addData("average2", pipeline.average2);
            telemetry.addData("color: ", Arrays.toString(Arrays.stream(pipeline.color).toArray()));
            telemetry.update();
            sleep(50);

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
                    mechOps.driveDistancePods(.2, 90, 13.5);
                    sleep(20);
                    mechOps.armLowIdle();
                    mechOps.wristPosition(params.WRIST_EXTEND);
                    sleep(1000);
                    mechOps.driveDistancePods(.2, 0, 7);
                    sleep(20);
                    mechOps.driveDistancePods(.2, 180, 1);
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
                    mechOps.driveDistancePods(.2, 90, 13.5);
                    sleep(20);
                    mechOps.armLowIdle();
                    mechOps.wristPosition(params.WRIST_EXTEND);
                    sleep(1000);
                    mechOps.driveDistancePods(.2, 0, 27);
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

    // 1280 X 720 Pixels
    OpenCvCamera webcam;
    SamplePipeline pipeline;

    public static class SamplePipeline extends OpenCvPipeline {
        private static final Scalar BLUE = new Scalar(255, 26, 0); //its red not blue


        Point Left1 = new Point(0, 50);
        Point Right1 = new Point(110, 180);

        Point Left2 = new Point(130, 50);
        Point Right2 = new Point(280, 180);

        //Point Left3 = new Point(212, 90);
        //Point Right3 = new Point(318, 110);

        Mat region1_Cb;
        Mat region2_Cb;
        //Mat region3_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();

        public volatile int average1, average2, average3, average;
        //int x = 120; // CHANGE THIS ******************************************
        private volatile TYPE type = TYPE.LEFT;

        public double[] color = {};

        private void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat input) {
            inputToCb(input);
            region1_Cb = Cb.submat(new Rect(Left1, Right1));
            region2_Cb = Cb.submat(new Rect(Left2, Right2));
            //region3_Cb = Cb.submat(new Rect(Left3, Right3));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);
            color = Cb.get(80, 130);

            average1 = (int) Core.mean(region1_Cb).val[0];
            average2 = (int) Core.mean(region2_Cb).val[0];
            //average3 = (int) Core.mean(region3_Cb).val[0];

            Imgproc.rectangle(input, Left1, Right1, BLUE, 2);
            Imgproc.rectangle(input, Left2, Right2, BLUE, 2);
            //Imgproc.rectan gle(input, Left3, Right3, BLUE, 2);
            if(average1 == average2) {
                type = TYPE.LEFT;
            } else if (((average1 >= 110 && average1 <= 126) && (average2 >= 110 && average2 <= 127))) {
//                    average = Math.min(average1, average2);
                type = TYPE.CENTER;
                autoState = State.CENTER;
            } else if ((average1 >= 126 && average1 <= 140) && (average2 >= 120 && average2 <= 140)) {
                type = TYPE.RIGHT;
                autoState = State.RIGHT;

            } else {
                type = TYPE.LEFT;
                autoState = State.LEFT;

            }
            //average = Math.max(average, average3);

            /*if (average == average1) {
                type = TYPE.LEFT;
            }
            else if (average == average2) {
                type = TYPE.CENTER;
            }
            else if (average == average3) {
                type = TYPE.RIGHT;
            }*/
            return input;
        }

        public TYPE getType() {
            return type;
        }

        public int getAverage() {
            return average;
        }

        public enum TYPE {
            RIGHT, CENTER, LEFT
        }

    }
}

