package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;

import java.util.List;

@Config
@TeleOp(name = "Broken Bot", group = "Development")
//@Disabled

public class BrokenBot extends LinearOpMode {
    private final static HWProfile robot = new HWProfile();
    public double servoPos = 1;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        boolean fieldCentric = true;
        LinearOpMode opMode = this;

        dashboard = FtcDashboard.getInstance();
        TelemetryPacket dashTelemetry = new TelemetryPacket();

        robot.init(hardwareMap);
        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);

        telemetry.addData("Ready to Run: ", "GOOD LUCK");
        telemetry.update();


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        waitForStart();

        while (opModeIsActive()) {
            robot.launcherServo.setPosition(servoPos);

            if(gamepad2.dpad_up) {
                robot.motorLF.setPower(1);
                telemetry.addData("motorLF spinning", "");
            } else {
                robot.motorLF.setPower(0);
            }
            if(gamepad2.dpad_down) {
                robot.motorLR.setPower(1);
                telemetry.addData("motorLR spinning", "");
            } else {
                robot.motorLR.setPower(0);
            }
            if(gamepad2.dpad_left) {
                robot.motorRR.setPower(1);
                telemetry.addData("motorRR spinning", "");
            } else {
                robot.motorRR.setPower(0);
            }
            if(gamepad2.dpad_right) {
                robot.motorRF.setPower(1);
                telemetry.addData("motorRF spinning", "");
            } else {
                robot.motorRF.setPower(0);
            }

            // Provide user feedback

/*            telemetry.addData("IMU Angle X = ", robot.imu.getAngles()[0]);
            telemetry.addData("IMU Angle Y = ", robot.imu.getAngles()[1]);
            telemetry.addData("IMU Angle Z = ", robot.imu.getAngles()[2]);
            telemetry.addData("Left Stick X = ", gp1.getLeftX());
            telemetry.addData("Left Stick Y = ", gp1.getLeftY());
            telemetry.addData("Right Stick X = ", gp1.getRightX());
            telemetry.addData("Right Stick Y = ", gp1.getRightY());
            telemetry.update();*/

            servoPos = Range.clip(servoPos, 0, 1);

            // post telemetry to FTC Dashboard as well
            dashTelemetry.put("01 - IMU Angle X = ", robot.imu.getAngles()[0]);
            dashTelemetry.put("02 - IMU Angle Y = ", robot.imu.getAngles()[1]);
            dashTelemetry.put("03 - IMU Angle Z = ", robot.imu.getAngles()[2]);

            dashTelemetry.put("13 - motorLF encoder = ", robot.motorLF.getCurrentPosition());
            dashTelemetry.put("14 - motorLR encoder = ", robot.motorLR.getCurrentPosition());
            dashTelemetry.put("15 - motorRF encoder = ", robot.motorRF.getCurrentPosition());
            dashTelemetry.put("16 - motorRR encoder = ", robot.motorRR.getCurrentPosition());
            dashTelemetry.put("19 - L/R Odometer = ",robot.motorRR.getCurrentPosition());
            dashTelemetry.put("20 - F/B Odometer = ",robot.motorLF.getCurrentPosition());
            dashboard.sendTelemetryPacket(dashTelemetry);

            telemetry.addData("Claw Axis Servo = ", servoPos);
            telemetry.addData("14 - motorLF encoder = ", robot.motorLF.getCurrentPosition());
            telemetry.addData("15 - motorLRF encoder = ", robot.motorLR.getCurrentPosition());
            telemetry.addData("16 - motorRF encoder = ", robot.motorRF.getCurrentPosition());
            telemetry.addData("17 - motorRR encoder = ", robot.motorRR.getCurrentPosition());
            telemetry.update();

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()

}       // end of BrokenBot class