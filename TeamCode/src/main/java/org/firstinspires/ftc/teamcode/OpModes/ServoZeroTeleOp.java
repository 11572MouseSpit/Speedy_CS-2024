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

@Config
@TeleOp(name = "Servo Zero TeleOp", group = "Development")

public class ServoZeroTeleOp extends LinearOpMode {
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

            // CLAW WRIST ZERO
            if(gamepad1.a) {
                robot.servoClawWrist.setPosition(params.CLAW_WRIST_ZERO);
            }
            telemetry.addLine("Gamepad1 A: Zero Wrist Servo");

            // ARM SERVOS ZERO
            if(gamepad1.b) {
                robot.servoArmActuatorA.setPosition(params.ARM_ZERO_POS);
                robot.servoArmActuatorB.setPosition(1 - params.ARM_ZERO_POS);
            }

            // ARM EXTEND ZERO
            if(gamepad1.x) {
                robot.servoArmExtend.setPosition(params.ARM_EXTEND_IN);
            }
            telemetry.addLine("Gamepad1 B: Zero Arm Servos");
            telemetry.update();


        }   // end of while(opModeIsActive)
    }   // end of runOpMode()

}       // end of BrokenBot class