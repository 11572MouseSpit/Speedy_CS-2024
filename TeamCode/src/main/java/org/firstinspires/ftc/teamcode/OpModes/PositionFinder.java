package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.Hardware.RRHWProfile;
import org.firstinspires.ftc.teamcode.Libs.RRMechOps;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@TeleOp(name = "Odometry Testing non RR", group = "Competition")

public class PositionFinder extends LinearOpMode {

    RRHWProfile robot = new RRHWProfile();
    private LinearOpMode opMode = this;
    private Params params = new Params();
    RRMechOps mechOps = new RRMechOps(robot, opMode, params);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, true);

        robot.parOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.parOdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.perpOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.perpOdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        double parStart = robot.parOdo.getCurrentPosition();
        double perpStart = robot.perpOdo.getCurrentPosition();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                parStart = robot.parOdo.getCurrentPosition();
                perpStart = robot.perpOdo.getCurrentPosition();
            }

            telemetry.addData("distance", mechOps.calcOdoDistance(parStart, perpStart));
            telemetry.addData("heading (deg)", mechOps.getZAngle());
            telemetry.addLine("Press A to reset position");
            telemetry.update();
        }
    }
}
