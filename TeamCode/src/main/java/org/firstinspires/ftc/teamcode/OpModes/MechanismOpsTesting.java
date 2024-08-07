package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.Hardware.RRHWProfile;
import org.firstinspires.ftc.teamcode.Libs.RRMechOps;

import java.util.List;

@Config
@TeleOp(name = "MechOps Testing", group = "Development")
//@Disabled

public class MechanismOpsTesting extends LinearOpMode {
    private final static RRHWProfile robot = new RRHWProfile();
    public double servoPos = 1;
    int liftPos = 0;
    boolean canOpen = true;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        boolean fieldCentric = true;
        LinearOpMode opMode = this;
        Params params = new Params();
        RRMechOps mechOps = new RRMechOps(robot, opMode, params);

        dashboard = FtcDashboard.getInstance();
        TelemetryPacket dashTelemetry = new TelemetryPacket();

        robot.init(hardwareMap, true);
        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);

        telemetry.addData("Ready to Run: ", "GOOD LUCK");
        telemetry.update();


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.dpad_down) {
                mechOps.initForAuto();
            }

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
            
            if(gamepad1.right_trigger > .1) {
                liftPos += 25;
            } else if(gamepad1.left_trigger > .1) {
            	liftPos -= 25;
            }

            if(gamepad2.right_trigger > .1) {
            } else if (gamepad2.left_trigger > .1) {
            }

            if(gamepad1.right_bumper) {
                mechOps.clawRightClose();
            } else if(gamepad1.left_bumper) {
                mechOps.clawRightOpen();
            }

//            liftPos = Range.clip(liftPos, 0, robot.LIFT_MAX_POS);

            robot.motorLift.setTargetPosition(liftPos);
            robot.motorLift.setPower(1);

            // Provide user feedback

            /**************************************************************************
             *
             *
             *                      MechOps Testing
             *
             *
             *************************************************************************/

            if(gamepad1.a){
                telemetry.addData("Lift Reset", "");
                mechOps.liftReset();
            }

            if(gamepad1.b){
                telemetry.addData("Lift Low Position", "");
                mechOps.liftPosition(params.LIFT_LOW_2_POSITION);
            }

            if(gamepad1.x){
                telemetry.addData("Lift Mid Position", "");
                mechOps.liftPosition(params.LIFT_MID_POSITION);
            }

            if(gamepad1.y){
                telemetry.addData("Lift High Position", "");
                mechOps.liftPosition(params.LIFT_HIGH_POSITION);
            }

            if(gamepad2.a){
                telemetry.addData("Open Left Claw", "");
                telemetry.addData("Open Right Claw", "");
                mechOps.clawLeftOpen();
                mechOps.clawRightOpen();
            }

            if(gamepad2.b){
                telemetry.addData("Close Left Claw", "");
                telemetry.addData("Close Right Claw", "");
                mechOps.clawLeftClose();
                mechOps.clawRightClose();
            }


            servoPos = Range.clip(servoPos, 0, 1);

            // post telemetry to FTC Dashboard as well
            dashTelemetry.put("13 - motorLF encoder = ", robot.motorLF.getCurrentPosition());
            dashTelemetry.put("14 - motorLR encoder = ", robot.motorLR.getCurrentPosition());
            dashTelemetry.put("15 - motorRF encoder = ", robot.motorRF.getCurrentPosition());
            dashTelemetry.put("16 - motorRR encoder = ", robot.motorRR.getCurrentPosition());
            dashTelemetry.put("19 - L/R Odometer = ",robot.motorRR.getCurrentPosition());
            dashTelemetry.put("20 - F/B Odometer = ",robot.motorLF.getCurrentPosition());
            dashboard.sendTelemetryPacket(dashTelemetry);

            telemetry.addData("Claw Axis Servo = ", servoPos);
            telemetry.addData("liftPos: ", liftPos);
            telemetry.addData("lift motor encoder: ", robot.motorLift.getCurrentPosition());
            telemetry.addData("14 - motorLF encoder = ", robot.motorLF.getCurrentPosition());
            telemetry.addData("15 - motorLRF encoder = ", robot.motorLR.getCurrentPosition());
            telemetry.addData("16 - motorRF encoder = ", robot.motorRF.getCurrentPosition());
            telemetry.addData("17 - motorRR encoder = ", robot.motorRR.getCurrentPosition());
            telemetry.update();

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()

}       // end of BrokenBot class
