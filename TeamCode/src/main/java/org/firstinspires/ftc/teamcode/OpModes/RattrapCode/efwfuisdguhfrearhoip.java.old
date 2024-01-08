package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

@TeleOp
public class efwfuisdguhfrearhoip extends LinearOpMode {
    // 1280 X 720 Pixels
    OpenCvCamera webcam;
    SamplePipeline pipeline;


    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new SamplePipeline();
        webcam.setPipeline(pipeline);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Type", pipeline.getType());
            telemetry.addData("Average", pipeline.getAverage());
            telemetry.addData("average1", pipeline.average1);
            telemetry.addData("average2", pipeline.average2);
            telemetry.addData("color: ", Arrays.toString(Arrays.stream(pipeline.color).toArray()));
            telemetry.update();
            sleep(50);
        }
    }

    public static class SamplePipeline extends OpenCvPipeline {
        private static final Scalar BLUE = new Scalar(0, 0, 255);


        Point Left1 = new Point(30, 50);
        Point Right1 = new Point(100, 180);

        Point Left2 = new Point(230, 130);
        Point Right2 = new Point(280, 160);

        //Point Left3 = new Point(212, 90);
        //Point Right3 = new Point(318, 110);

        Mat region1_Cb;
        Mat region2_Cb;
        //Mat region3_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();

        public volatile int average1, average2, average3, average;
        //int x = 120; // CHANGE THIS ******************************************
        private volatile TYPE type = TYPE.CENTER;

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

                if (((average1 >= 137 && average1 <= 150) && (average2 >= 140 && average2 <= 160))) {
//                    average = Math.min(average1, average2);
                    type = TYPE.CENTER;
                }
                else if((average1 >= 130 && average1 <= 140) && (average2 >= 165 && average2 <= 200)) {
                    type = TYPE.RIGHT;
                } else {
                    type = TYPE.LEFT;
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