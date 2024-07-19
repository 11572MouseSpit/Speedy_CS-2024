package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.Hardware.RRHWProfile;
import org.firstinspires.ftc.teamcode.Libs.RRMechOps;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            RRHWProfile hwProfile = new RRHWProfile();
            LinearOpMode opMode = this;
            hwProfile.init(hardwareMap, false);
            Params params = new Params();
            RRMechOps mechOps = new RRMechOps(hwProfile, opMode, params);
            boolean firstRun = true;

            mechOps.clawLeftClose();
            mechOps.clawRightClose();

            waitForStart();

            double headingOffset = 0;

            while (opModeIsActive()) {
                if(firstRun) {
                    drive.localizer.resetOdo();
                    drive.updatePoseEstimate();
                    firstRun = false;
                    telemetry.addLine("update odo");
                    telemetry.update();
                }

                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble() + headingOffset));
                telemetry.addData("heading actual (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                telemetry.addData("heading offset", Math.toDegrees(headingOffset));
                telemetry.update();
            }
        }
    }
}
