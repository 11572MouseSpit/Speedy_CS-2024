package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.Hardware.RRHWProfile;
import org.firstinspires.ftc.teamcode.Libs.RRMechOps;

import java.util.HashMap;

@Config
@TeleOp(name = "Motor PID Tuner", group = "Development")

public class PIDTuner extends LinearOpMode {
    private final static RRHWProfile robot = new RRHWProfile();
    private final static Params params = new Params();
    private LinearOpMode opMode = this;
    private static HashMap<String, Double> editableVariableNames = new HashMap<String, Double>();
    private final static String[] motors = {"armMotor", "motorLift"};
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    double liftPos = 0;
    @Override
    public void runOpMode() {

        robot.init(hardwareMap, true);
        RRMechOps mechOps = new RRMechOps(robot, opMode, params);
        double parStart = robot.parOdo.getCurrentPosition();
        double perpStart = robot.perpOdo.getCurrentPosition();
        int targetPos = 0;
        boolean targetPosSelected = false;
        double armPos = 0;
        int selectedVariable = 0;
        editableVariableNames.put("kP", 0.0);
        editableVariableNames.put("kI", 0.0);
        editableVariableNames.put("kD", 0.0);

        telemetry.addData("Ready to Run: ", "GOOD LUCK");
        telemetry.update();

        boolean isMotorSelected = false;
        String selectedMotorName = "";
        DcMotorEx selectedMotor = null;
        int motorSelectionIndex = 0;
        boolean dpadLeftCooldown = false;
        boolean dpadRightCooldown = false;
        boolean aBtnCooldown = false;
        boolean bBtnCooldown = false;
        String[] variableList = editableVariableNames.keySet().toArray(new String[0]);

        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addLine("DO NOT RUN IF YOU DO NOT KNOW WHAT PID IS!");
        telemetry.addLine("DO NOT RUN IF YOU DO NOT KNOW WHAT PID IS!");
        telemetry.addLine("DO NOT RUN IF YOU DO NOT KNOW WHAT PID IS!");
        telemetry.addLine("DO NOT RUN IF YOU DO NOT KNOW WHAT PID IS!");
        telemetry.addLine("DO NOT RUN IF YOU DO NOT KNOW WHAT PID IS!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            // Provide user feedback

            /**************************************************************************
             *
             *
             *                          Motor PID Tuner
             *
             *
             *************************************************************************/

            if(isMotorSelected == false) {
                telemetry.addLine("DO NOT RUN IF YOU DO NOT KNOW WHAT PID IS!");
                telemetry.addLine("DO NOT RUN IF YOU DO NOT KNOW WHAT PID IS!");
                telemetry.addLine("DO NOT RUN IF YOU DO NOT KNOW WHAT PID IS!");
                telemetry.addLine("DO NOT RUN IF YOU DO NOT KNOW WHAT PID IS!");
                telemetry.addLine("DO NOT RUN IF YOU DO NOT KNOW WHAT PID IS!");
                telemetry.addData("Select Motor (dpad left/dpad right): ", motors[motorSelectionIndex]);
                telemetry.addLine("Press A (X on playstation) to continue");

                if(gamepad1.dpad_left && !dpadLeftCooldown) { // select motor to tune
                    motorSelectionIndex--;
                    if(motorSelectionIndex < 0) {
                        motorSelectionIndex = motors.length - 1;
                    }
                    dpadLeftCooldown = true;
                } else if(gamepad1.dpad_right && !dpadRightCooldown) {
                    motorSelectionIndex++;
                    if(motorSelectionIndex > motors.length - 1) {
                        motorSelectionIndex = 0;
                    }

                    dpadRightCooldown = true;
                }

                if(gamepad1.a) {
                    aBtnCooldown = true;
                    selectedMotorName = motors[motorSelectionIndex];
                    isMotorSelected = true;

                    selectedMotor = robot.hwMap.get(DcMotorEx.class, selectedMotorName);
                    selectedMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    selectedMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    selectedMotor.setTargetPosition(0);
                    selectedMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    selectedMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    selectedMotor.setPower(0);

                    editableVariableNames.put("kP", selectedMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).p);
                    editableVariableNames.put("kI", selectedMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).i);
                    editableVariableNames.put("kD", selectedMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).d);
                }
            } else if(targetPosSelected == false) {
                telemetry.addData("Selected Target Position (dpad left/right): ", targetPos);
                telemetry.addLine("Press A (X on playstation) to continue");

                if(gamepad1.dpad_left && !dpadLeftCooldown) {
                    targetPos -= 10;
                    dpadLeftCooldown = true;
                } else if(gamepad1.dpad_right && !dpadRightCooldown) {
                    targetPos += 10;
                    dpadRightCooldown = true;
                }
                if(gamepad1.a && !aBtnCooldown) {
                    targetPosSelected = true;
                    aBtnCooldown = true;
                }
            } else { // do PID tuning
                telemetry.addData("kP", editableVariableNames.get("kP"));
                telemetry.addData("kI", editableVariableNames.get("kI"));
                telemetry.addData("kD", editableVariableNames.get("kD"));
                telemetry.addLine("Press B to run the selected motor.");
                telemetry.addLine("Press A to cycle the variable to change.");
                telemetry.addLine("Press DPad Left/Right to change the selected variable.");
                telemetry.addLine("");
                telemetry.addData("Selected Variable", variableList[selectedVariable]);
                telemetry.addData("Motor Encoder", selectedMotor.getCurrentPosition());

                if(gamepad1.a && !aBtnCooldown) {
                    aBtnCooldown = true;
                    selectedVariable++;

                    if(selectedVariable > variableList.length - 1) selectedVariable = 0;
                    if(selectedVariable < 0) selectedVariable = variableList.length - 1;
                }

                if(gamepad1.b && !bBtnCooldown) {
                    bBtnCooldown = true;
                    if(selectedMotor.getTargetPosition() == targetPos) {
                        selectedMotor.setTargetPosition(0);
                    } else {
                        selectedMotor.setTargetPosition(targetPos);
                    }

                    selectedMotor.setPower(params.PID_TUNE_MOTOR_POWER);
                }
            }

            telemetry.update();
            if(!gamepad1.dpad_left) dpadLeftCooldown = false;
            if(!gamepad1.dpad_right) dpadRightCooldown = false;
            if(!gamepad1.a) aBtnCooldown = false;
            if(!gamepad1.b) bBtnCooldown = false;
        }   // end of while(opModeIsActive)
    }   // end of runOpMode()

}       // end of BrokenBot class