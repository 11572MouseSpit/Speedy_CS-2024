package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.Hardware.RRHWProfile;
import org.firstinspires.ftc.teamcode.Libs.RRMechOps;

import java.nio.ByteBuffer;

@Config
@TeleOp(name = "Gamepad Tester", group = "Development")

public class GamepadTester extends LinearOpMode {
    private final static RRHWProfile robot = new RRHWProfile();
    private final static Params params = new Params();
    private LinearOpMode opMode = this;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    double liftPos = 0;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, true);
        RRMechOps mechOps = new RRMechOps(robot, opMode, params);
        double parStart = robot.parOdo.getCurrentPosition();
        double perpStart = robot.perpOdo.getCurrentPosition();
        double armPos = 0;

        telemetry.addData("Ready to Run: ", "GOOD LUCK");
        telemetry.update();

        boolean gamepad1Active = true;

        waitForStart();

        ByteBuffer gamepad1Buffer = ByteBuffer.wrap(gamepad1.toByteArray());

        while (opModeIsActive()) {

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            // Provide user feedback

            /**************************************************************************
             *
             *
             *                           Servo Zero TeleOp
             *
             *
             *************************************************************************/

            telemetry.addLine("Gamepad1: " + gamepad1.toByteArray());
            telemetry.update();


        }   // end of while(opModeIsActive)
    }   // end of runOpMode()

}       // end of BrokenBot class