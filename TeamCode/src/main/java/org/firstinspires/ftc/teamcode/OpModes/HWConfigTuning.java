package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.Hardware.RRHWProfile;

@Config
@TeleOp(name = "Hardware Config Tuning", group = "Development")

public class HWConfigTuning extends LinearOpMode {
    private final static RRHWProfile robot = new RRHWProfile();
    private final static Params params = new Params();
    private LinearOpMode opMode = this;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    public static int P01_LIFT_RESET = params.LIFT_RESET;
    public static int P02_LIFT_LOW_POSITION = params.LIFT_LOW_POSITION;
    public static double P03_CLAW_LEFT_OPEN = params.CLAW_LEFT_OPEN;
    public static double P04_CLAW_LEFT_CLOSE = params.CLAW_LEFT_CLOSE;
    public static double P05_CLAW_RIGHT_OPEN = params.CLAW_RIGHT_OPEN;
    public static double P06_CLAW_RIGHT_CLOSE = params.CLAW_RIGHT_CLOSE;
    public static double P07_BUCKET_RESET = params.BUCKET_RESET;
    public static double P08_BUCKET_SCORE = params.BUCKET_SCORE;
    public static double P09_DRONE_LOAD = params.DRONE_LOAD;
    public static double P10_DRONE_FIRE = params.DRONE_FIRE;
    public static double P11_ARM_LEFT_EXTEND = params.ARM_LEFT_EXTEND;
    public static double P12_ARM_LEFT_IDLE = params.ARM_LEFT_IDLE;
    public static double P13_ARM_LEFT_RESET = params.ARM_LEFT_RESET;
    public static double P14_ARM_RIGHT_EXTEND = params.ARM_RIGHT_EXTEND;
    public static double P15_ARM_RIGHT_IDLE = params.ARM_RIGHT_IDLE;
    public static double P16_ARM_RIGHT_RESET = params.ARM_RIGHT_RESET;
    public static double P17_WRIST_RESET = params.WRIST_RESET;
    public static double P18_WRIST_LOAD_PIXELS = params.WRIST_LOAD_PIXELS;
    public static double P19_WRIST_EXTEND = params.WRIST_EXTEND;
    public static double P20_SLIDE_LEFT_EXTEND = params.SLIDE_LEFT_EXTEND;
    public static double P21_SLIDE_LEFT_RESET = params.SLIDE_LEFT_RESET;
    public static double P22_SLIDE_RIGHT_EXTEND = params.SLIDE_RIGHT_EXTEND;
    public static double P23_SLIDE_RIGHT_RESET = params.SLIDE_RIGHT_RESET;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("Ready to Run: ", "GOOD LUCK");
        telemetry.update();

        boolean gamepad1Active = true;

        waitForStart();

        while (opModeIsActive()) {

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            // Provide user feedback

            /**************************************************************************
             *
             *
             *                      MechOps Testing
             *
             *
             *************************************************************************/

            if(gamepad1.a){
                robot.servoClawLeft.setPosition(P03_CLAW_LEFT_OPEN);
            }

            if(gamepad1.b){
                robot.servoClawLeft.setPosition(P04_CLAW_LEFT_CLOSE);
            }

            if(gamepad1.x){
                robot.servoClawRight.setPosition(P05_CLAW_RIGHT_OPEN);
            }

            if(gamepad1.y){
                robot.servoClawRight.setPosition(P06_CLAW_RIGHT_CLOSE);
            }

            if(gamepad1.dpad_left){
                robot.servoSlideLeft.setPosition(P21_SLIDE_LEFT_RESET);
            }

            if(gamepad1.dpad_right){
                robot.servoSlideLeft.setPosition(P20_SLIDE_LEFT_EXTEND);
            }

            if(gamepad1.dpad_up){
                robot.servoSlideRight.setPosition(P22_SLIDE_RIGHT_EXTEND);
            }

            if(gamepad1.dpad_down){
                robot.servoSlideRight.setPosition(P23_SLIDE_RIGHT_RESET);
            }

            if(gamepad1.right_trigger>0) {
                robot.motorLift.setTargetPosition(P02_LIFT_LOW_POSITION);
                robot.motorLift.setPower(params.LIFT_POWER_UP);
            }

            if(gamepad1.left_trigger > 0 ){
                robot.motorLift.setTargetPosition(P01_LIFT_RESET);
                robot.motorLift.setPower(params.LIFT_POWER_DOWN);
            }

            if(gamepad2.left_bumper){
                robot.servoDrone.setPosition(P09_DRONE_LOAD);
            }

            if(gamepad2.right_bumper){
                robot.servoDrone.setPosition(P10_DRONE_FIRE);
            }

            if(gamepad2.x){
                robot.servoBucket.setPosition(P08_BUCKET_SCORE);
            }

            if(gamepad2.a){
                robot.servoBucket.setPosition(P07_BUCKET_RESET);
            }

            if(gamepad2.y){
                robot.servoBucket.setPosition(P08_BUCKET_SCORE);
            }

            if(gamepad2.dpad_up){
                robot.servoArmRight.setPosition(P14_ARM_RIGHT_EXTEND);
            }

            if(gamepad2.dpad_down){
                robot.servoArmRight.setPosition(P16_ARM_RIGHT_RESET);
            }

            if(gamepad2.dpad_left){
                robot.servoArmLeft.setPosition(P13_ARM_LEFT_RESET);
            }

            if(gamepad2.dpad_right){
                robot.servoArmLeft.setPosition(P11_ARM_LEFT_EXTEND);
            }

            if(gamepad1.right_stick_button || gamepad1.left_stick_button || gamepad2.right_stick_button || gamepad2.left_stick_button){
                gamepad1Active = false;
            }
            if(gamepad1Active) {
                telemetry.addData("Gamepad1", "Controls");
                telemetry.addData("Gamepad1.A - Left Claw Open = ", P03_CLAW_LEFT_OPEN);
                telemetry.addData("Gamepad1.B - Left Claw Close = ", P04_CLAW_LEFT_CLOSE);
                telemetry.addData("Gamepad1.X - Right Claw Open = ", P05_CLAW_RIGHT_OPEN);
                telemetry.addData("Gamepad1.Y - Right Claw Close = ", P06_CLAW_RIGHT_CLOSE);
                telemetry.addData("Gamepad1.dpad_right - Slide Left Extend = ", P20_SLIDE_LEFT_EXTEND);
                telemetry.addData("Gamepad1.dpad_left - Slide Left Reset = ", P21_SLIDE_LEFT_RESET);
                telemetry.addData("Gamepad1.dpad_up - Slide Right Extend = ", P22_SLIDE_RIGHT_EXTEND);
                telemetry.addData("Gamepad1.dpad_down - Slide Right Reset = ", P23_SLIDE_RIGHT_RESET);
                telemetry.addData("Gamepad1.right_trigger -Lift Low Position = ", P02_LIFT_LOW_POSITION);
                telemetry.addData("Gamepad1.left_trigger - Lift Reset = ", P01_LIFT_RESET);
            } else {
                telemetry.addData("Gamepad2", "Controls");
                telemetry.addData("Gamepad2.left_bumper -Servo Drone Load = ", P09_DRONE_LOAD);
                telemetry.addData("Gamepad2.right_bumper -Servo Drone Fire = ", P10_DRONE_FIRE);
                telemetry.addData("Gamepad2.y - Servo Bucket Dump = ", P08_BUCKET_SCORE);
                telemetry.addData("Gamepad2.a - Servo Bucket Reset = ", P07_BUCKET_RESET);
                telemetry.addData("Gamepad2.dpad_up - Left Arm Extend = ", P11_ARM_LEFT_EXTEND);
                telemetry.addData("Gamepad2.dpad_down - Left Arm Reset = ", P13_ARM_LEFT_RESET);
                telemetry.addData("Gamepad2.dpad_right - Right Arm Extend = ", P14_ARM_RIGHT_EXTEND);
                telemetry.addData("Gamepad2.dpad_left - Right Arm Reset = ", P16_ARM_RIGHT_RESET);

            }
            telemetry.update();

            packet.put("P01_LIFT_RESET = ", P01_LIFT_RESET);
            packet.put("P02_LIFT_LOW_POSITION = ", P02_LIFT_LOW_POSITION);
            packet.put("P03_CLAW_LEFT_OPEN = ", P03_CLAW_LEFT_OPEN);
            packet.put("P04_CLAW_LEFT_CLOSE = ", P04_CLAW_LEFT_CLOSE);
            packet.put("P05_CLAW_RIGHT_OPEN = ", P05_CLAW_RIGHT_OPEN);
            packet.put("P06_CLAW_RIGHT_CLOSE = ",P06_CLAW_RIGHT_CLOSE);
            packet.put("P06_BUCKET_RESET = ",P07_BUCKET_RESET);
            packet.put("P08_BUCKET_SCORE = ",P08_BUCKET_SCORE);
            packet.put("P09_DRONE_LOAD = ",P09_DRONE_LOAD);
            packet.put("P10_DRONE_FIRE = ",P10_DRONE_FIRE);
            packet.put("P11_ARM_LEFT_EXTEND = ",P11_ARM_LEFT_EXTEND);
            packet.put("P12_ARM_LEFT_IDLE = ",P12_ARM_LEFT_IDLE);
            packet.put("P13_ARM_LEFT_RESET = ",P13_ARM_LEFT_RESET);
            packet.put("P14_ARM_RIGHT_EXTEND = ",P14_ARM_RIGHT_EXTEND);
            packet.put("P15_ARM_RIGHT_IDLE = ",P15_ARM_RIGHT_IDLE);
            packet.put("P16_ARM_RIGHT_RESET = ",P16_ARM_RIGHT_RESET);
            packet.put("P17_WRIST_RESET = ",P17_WRIST_RESET);
            packet.put("P18_WRIST_LOAD_PIXELS = ",P18_WRIST_LOAD_PIXELS);
            packet.put("P19_WRIST_EXTEND = ",P19_WRIST_EXTEND);
            packet.put("P20_SLIDE_LEFT_EXTEND = ",P20_SLIDE_LEFT_EXTEND);
            packet.put("P21_SLIDE_LEFT_RESET = ",P21_SLIDE_LEFT_RESET);
            packet.put("P22_SLIDE_RIGHT_EXTEND = ",P22_SLIDE_RIGHT_EXTEND);
            packet.put("P23_SLIDE_RIGHT_RESET = ",P23_SLIDE_RIGHT_RESET);
            dashboard.sendTelemetryPacket(packet);


        }   // end of while(opModeIsActive)
    }   // end of runOpMode()

}       // end of BrokenBot class