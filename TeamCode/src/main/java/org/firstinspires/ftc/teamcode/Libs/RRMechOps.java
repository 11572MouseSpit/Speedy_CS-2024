package org.firstinspires.ftc.teamcode.Libs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.Hardware.RRHWProfile;

@Config
public class RRMechOps {

    private RRHWProfile robot;
    public LinearOpMode opMode;
    public Params params;
    public boolean bucketScored = false;
    public static double kP = 0.05;
    public static double kV = 0;
    public double secondMotorPower = 0;
    public double RRPower;
    public double RFPower;
    public double LRPower;
    public double LFPower;

    public boolean climbLeftHomed = false;
    public boolean climbRightHomed = false;

    public  static void main(String[] args) {

    }

    /*
     * Constructor method
     */
    public RRMechOps(RRHWProfile myRobot, LinearOpMode myOpMode, Params myParams){
        robot = myRobot;
        opMode = myOpMode;
        params = myParams;

    }   // close RRMechOps constructor Method

    public void liftPos(int liftPos) {
        liftPosition(liftPos);
    }


    public void motorsHalt() {
        robot.motorLF.setPower(0);
        robot.motorLR.setPower(0);
        robot.motorRF.setPower(0);
        robot.motorRR.setPower(0);
    }

    public double getZAngle(){
        return (robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        //return (-robot.imu.getAngularOrientation().firstAngle);
    }   // close getZAngle method


    public double calcDistance(double heading, double rfStart, double rrStart, double lfStart, double lrStart){

        double distanceTraveled = 0;
        double rfEncoder = 0;
        double lfEncoder = 0;
        double rrEncoder = 0;
        double lrEncoder = 0;

        rfEncoder = robot.motorRF.getCurrentPosition();
        lfEncoder = robot.motorLF.getCurrentPosition();
        rrEncoder = robot.motorRR.getCurrentPosition();
        lrEncoder = robot.motorLR.getCurrentPosition();

        if((heading == 0) || (heading == 180) || (heading == 90) || (heading == -90)) {
            distanceTraveled = ((Math.abs(rfStart - rfEncoder) + Math.abs(lfStart - lfEncoder)
                    + Math.abs(rrStart-rrEncoder) + Math.abs(lrStart - lrEncoder)) / 4) / (robot.DRIVE_TICKS_PER_INCH);
        }

        if ((heading == 90) || (heading == -90)){
            distanceTraveled = ((Math.abs(rfStart - rfEncoder) + Math.abs(lfStart - lfEncoder)
                    + Math.abs(rrStart-rrEncoder) + Math.abs(lrStart - lrEncoder)) / 4) / (robot.DRIVE_TICKS_PER_INCH);
        }

        return Math.abs(distanceTraveled);
    }

    public double calcOdoDistance(double parStart, double perpStart){

        double distanceTraveled = 0;
        double xValue = Math.abs((parStart - robot.parOdo.getCurrentPosition()) * params.PAR_IN_PER_TICK);
        double yValue = Math.abs((perpStart - robot.perpOdo.getCurrentPosition()) * params.PERP_IN_PER_TICK);

        distanceTraveled = Math.sqrt((xValue * xValue) + (yValue*yValue));

        return Math.abs(distanceTraveled);
    }
    public double calcParOdoDistance(double parStart){

        double distanceTraveled = 0;
        double xValue = Math.abs((parStart - robot.parOdo.getCurrentPosition()) * params.PAR_IN_PER_TICK);

//        distanceTraveled = Math.sqrt((xValue * xValue) + (yValue*yValue));

        return Math.abs(xValue);
    }

    public double calcPerpOdoDistance(double perpStart){

        double distanceTraveled = 0;
//        double xValue = Math.abs((parStart - robot.parOdo.getCurrentPosition()) * params.PAR_IN_PER_TICK);
        double yValue = Math.abs((perpStart - robot.perpOdo.getCurrentPosition()) * params.PERP_IN_PER_TICK);

//        distanceTraveled = Math.sqrt((xValue * xValue) + (yValue*yValue));

        return Math.abs(yValue);
    }

    public double PIDRotate(double targetAngle, double targetError){
        double integral = 0;
        ElapsedTime timeElapsed = new ElapsedTime();
        double startTime = timeElapsed.time();
        double totalTime;
        double error;
        double Cp = 0.006;
        double Ci = 0.003;
        double Cd = 0.00004;
        /* enable these for tuning
        double Cp = kP;
        double Ci = kI;
        double Cd = kD;
        double maxRotateSpeed = maxSpeed;
        double maxRotateSpeed = minSpeed;
         */
        double minRotateSpeed = 0.14;
        double maxRotateSpeed = 1;
        double rotationSpeed;
        double derivative = 0, lastError=0;

        // check to see how far the robot is rotating to decide which gyro sensor value to use
        if(targetAngle > 90 || targetAngle < -90){
            error = gyro360(targetAngle) - targetAngle;
        } else {
            error = getZAngle() - targetAngle;
        }

        // nested while loops are used to allow for a final check of an overshoot situation
        while ((Math.abs(error) >= targetError) && opMode.opModeIsActive()) {
            while ((Math.abs(error) >= targetError) && opMode.opModeIsActive()) {
                derivative = lastError - error;
                rotationSpeed = ((Cp * error) + (Ci * integral) + (Cd * derivative));
                lastError = error;

                // Clip motor speed
                rotationSpeed = Range.clip(rotationSpeed, -maxRotateSpeed, maxRotateSpeed);

                if ((rotationSpeed > -0.25) && (rotationSpeed < 0)) {
                    rotationSpeed = -minRotateSpeed;
                } else if ((rotationSpeed < 0.25) && (rotationSpeed > 0)) {
                    rotationSpeed = minRotateSpeed;
                }

                RFPower = rotationSpeed;
                LFPower = -rotationSpeed;
                LRPower = -rotationSpeed;
                RRPower = rotationSpeed;

                setDrivePower(RFPower, LFPower, LRPower, RRPower);

                opMode.idle();

                // check to see how far the robot is rotating to decide which gyro sensor value to use
                if (targetAngle > 90 || targetAngle < -90) {
                    error = gyro360(targetAngle) - targetAngle;
                } else {
                    error = getZAngle() - targetAngle;
                }

            }   // end of while Math.abs(error)
            setDrivePower(0,0,0,0);
            maxRotateSpeed = maxRotateSpeed / 2;
            opMode.sleep(10);
//            opMode.idle();

            // Perform a final calc on the error to confirm that the robot didn't overshoot the
            // target position after the last measurement was taken.
//            opMode.sleep(5);
            if (targetAngle > 90 || targetAngle < -90) {
                error = gyro360(targetAngle) - targetAngle;
            } else {
                error = getZAngle() - targetAngle;
//                error = getZAngle() - targetAngle;
            }
        }

        // shut off the drive motors
        motorsHalt();

        totalTime = timeElapsed.time() - startTime;
        // post telemetry to FTC Dashboard

        return(getZAngle() - targetAngle);        // return the rotate error value
    }   //end of the PIDRotate Method


    public void setDrivePower(double RF, double LF, double LR, double RR){
        robot.motorRF.setPower(RF);
        robot.motorLF.setPower(LF);
        robot.motorLR.setPower(LR);
        robot.motorRR.setPower(RR);
    }   // end of the setDrivePower method

    public double gyro360(double targetAngle){
        double currentZ = getZAngle();
        double rotationalAngle;

        if (targetAngle > 0){
            if ((currentZ >= 0) && (currentZ <= 180)) {
                rotationalAngle = currentZ;
            } else {
                rotationalAngle = 180 + (180 + currentZ);
            }// end if(currentZ <=0) - else
        } else {
            if ((currentZ <= 0) && (currentZ >= -180)) {
                rotationalAngle = currentZ;
            } else {
                rotationalAngle = -180 - (180 - currentZ);
            }   // end if(currentZ <=0) - else
        }   // end if(targetAngle >0)-else

        return rotationalAngle;
    }   // end method gyro360

    public void driveDistancePods(double power, double heading, double distance) {
        double initZ = getZAngle();
        double currentZ = 0;
        double zCorrection = 0;
        boolean active = true;
        double strafeFactor = 1;
        boolean correction = false;
        ElapsedTime correctionTime = new ElapsedTime();
        ElapsedTime Timeout = new ElapsedTime();
        Timeout.reset();
        Timeout.startTime();
        double theta = Math.toRadians(90 + heading);
        double parStart = 0;
        double perpStart = 0;
        double distanceTraveled = 0;
        int reverse = -1;
        double maxPower = 1;
        double minPower = -1;

        parStart = robot.parOdo.getCurrentPosition();
        perpStart = robot.perpOdo.getCurrentPosition();

        if (heading == 90 || heading == -90){
            strafeFactor = robot.STRAFE_FACTOR;
        }



        while(opMode.opModeIsActive() && active) {

            RFPower = reverse * power * (Math.sin(theta) + Math.cos(theta));
            LFPower = reverse * power * (Math.sin(theta) - Math.cos(theta));
            LRPower = reverse * power * (Math.sin(theta) + Math.cos(theta));
            RRPower = reverse * power * (Math.sin(theta) - Math.cos(theta));

            if (initZ > 170 || initZ < -170){
                currentZ = gyro360(0);      // always use 0 as the reference angle
            } else {
                currentZ = getZAngle();
            }
            if (currentZ != initZ){
                zCorrection = Math.abs(initZ - currentZ)/100;

                if (initZ < currentZ) {
                    RFPower = RFPower + zCorrection;
                    RRPower = RRPower + zCorrection;
                    LFPower = LFPower - zCorrection;
                    LRPower = LRPower - zCorrection;
                }
                if (initZ > currentZ) {
                    RFPower = RFPower - zCorrection;
                    RRPower = RRPower - zCorrection;
                    LFPower = LFPower + zCorrection;
                    LRPower = LRPower + zCorrection;
                }
            }   // end of if currentZ != initZ

            /*
             * Limit that value of the drive motors so that the power does not exceed 100%
             */
            RFPower = Range.clip(RFPower, minPower, maxPower);
            LFPower = Range.clip(LFPower, minPower, maxPower);
            RRPower = Range.clip(RRPower, minPower, maxPower);
            LRPower = Range.clip(LRPower, minPower, maxPower);

            /*
             * Apply power to the drive wheels
             */
            setDrivePower(RFPower, LFPower, LRPower, RRPower);

//            opMode.telemetry.addData("LFPower Start = ", lfStart);
//            opMode.telemetry.addData("Distance = ", distance);
            opMode.telemetry.addData("Heading = ", heading);
            opMode.telemetry.addData("Calculated Distance = ", calcOdoDistance(parStart, perpStart));
            opMode.telemetry.update();


            if(correctionTime.time() > 10 && correction == true) {
                active = false;
                break;
            }

            if(Timeout.time() > 7) {
                active = false;
                break;
            }

            distanceTraveled = calcOdoDistance(parStart, perpStart);
            if (distanceTraveled >= distance) {
                double diff = Math.abs(distanceTraveled) - Math.abs(distance);

                if (diff > .3) {
                    maxPower = .1;
                    minPower = -.1;
                    reverse = 1;
                    correction = true;

                    if (!correction) {
                        correctionTime.reset();
                    }
                } else if (diff < .2) {
                    active = false;
                } else {
                    maxPower = .1;
                    minPower = -.1;
                    reverse = 0;
                    correction = true;

                    if (!correction) {
                        correctionTime.reset();
                    }
                }
            }

            //opMode.idle();

        }   // end of while loop

        motorsHalt();

    }   // close driveDistance method

    public void driveDistance(double power, double heading, double distance) {
        double initZ = getZAngle();
        double currentZ = 0;
        double zCorrection = 0;
        boolean active = true;
        double strafeFactor = 1;

        double theta = Math.toRadians(90 + heading);
        double lfStart = 0;
        double lrStart = 0;
        double secondMotorPower = 1;
        double rfStart = 0;
        double rrStart = 0;

        lfStart = robot.motorLF.getCurrentPosition();
        lrStart = robot.motorLR.getCurrentPosition();
        rfStart = robot.motorRF.getCurrentPosition();
        rrStart = robot.motorRR.getCurrentPosition();

        if (heading == 90 || heading == -90){
            strafeFactor = robot.STRAFE_FACTOR;
        }

        while(opMode.opModeIsActive() && active) {

            RFPower = power * (Math.sin(theta) + Math.cos(theta));
            LFPower = power * (Math.sin(theta) - Math.cos(theta));
            LRPower = power * (Math.sin(theta) + Math.cos(theta));
            RRPower = power * (Math.sin(theta) - Math.cos(theta));

            if (initZ > 170 || initZ < -170){
                currentZ = gyro360(0);      // always use 0 as the reference angle
            } else {
                currentZ = getZAngle();
            }
            if (currentZ != initZ){
                zCorrection = Math.abs(initZ - currentZ)/100;

                if (initZ < currentZ) {
                    RFPower = RFPower + zCorrection;
                    RRPower = RRPower + zCorrection;
                    LFPower = LFPower - zCorrection;
                    LRPower = LRPower - zCorrection;
                }
                if (initZ > currentZ) {
                    RFPower = RFPower - zCorrection;
                    RRPower = RRPower - zCorrection;
                    LFPower = LFPower + zCorrection;
                    LRPower = LRPower + zCorrection;
                }
            }   // end of if currentZ != initZ

            /*
             * Limit that value of the drive motors so that the power does not exceed 100%
             */
            RFPower = Range.clip(RFPower, -1,1);
            LFPower = Range.clip(LFPower, -1,1);
            RRPower = Range.clip(RRPower, -1,1);
            LRPower = Range.clip(LRPower, -1,1);

            /*
             * Apply power to the drive wheels
             */
            setDrivePower(RFPower, LFPower, LRPower, RRPower);
            /*
            opMode.telemetry.addData("LFPower Start = ", lfStart);
            opMode.telemetry.addData("Distance = ", distance);
            opMode.telemetry.addData("Heading = ", heading);
            opMode.telemetry.addData("Calculated Distance = ", calcDistance(heading, rfStart, rrStart, lfStart, lrStart));
            opMode.telemetry.update();
             */

            if(calcDistance(heading, rfStart, rrStart, lfStart, lrStart) >= (distance * strafeFactor)) active = false;
            opMode.idle();

        }   // end of while loop

        motorsHalt();

    }   // close driveDistance method


    public void clawLeftOpen(){
        robot.servoClawLeft.setPosition(params.CLAW_LEFT_OPEN);
    }
    public void clawLeftOpenAuto(){
        robot.servoClawLeft.setPosition(params.CLAW_LEFT_OPEN_AUTO);
    }

    public void clawLeftClose(){
        robot.servoClawLeft.setPosition(params.CLAW_LEFT_CLOSE);
    }

    public void clawRightOpen(){
        robot.servoClawRight.setPosition(params.CLAW_RIGHT_OPEN);
    }
    public void clawRightOpenAuto(){
        robot.servoClawRight.setPosition(params.CLAW_RIGHT_OPEN_AUTO);
    }

    public void clawRightClose(){
        robot.servoClawRight.setPosition(params.CLAW_RIGHT_CLOSE);
    }





    public void clawLeftOpenBoard(){
        robot.servoClawLeft.setPosition(params.CLAW_LEFT_OPEN_BOARD);
    }


    public void clawRightOpenBoard(){
        robot.servoClawRight.setPosition(params.CLAW_RIGHT_OPEN_BOARD);
    }

    public void droneLoad() {
        robot.servoDrone.setPosition(params.DRONE_LOAD);
    }

    public void armUp() {
        params.armDeployed = true;
        robot.armMotor.setTargetPosition(params.ARM_UP_POS);
        robot.armMotor.setPower(params.ARM_IDLE_POWER);
    }

    public void armUpAuto() {
        robot.armMotor.setTargetPosition(params.ARM_UP_AUTO_POS);
        robot.armMotor.setPower(params.ARM_IDLE_POWER);
        params.armDeployed = true;
    }

    public void armHighScore() {
        robot.armMotor.setTargetPosition(params.ARM_HIGH_SCORE_POS);
        robot.armMotor.setPower(params.ARM_IDLE_POWER);
        params.armDeployed = true;
    }

    public void intakeDeploy() {
        robot.servoIntakeActuator.setPosition(params.INTAKE_DEPLOY_POS);
        robot.servoIntake.setPosition(params.INTAKE_SPEED);
    }

    public void intakeDeployStack() {
        robot.servoIntakeActuator.setPosition(params.INTAKE_DEPLOY_STACK_POS);
        robot.servoIntake.setPosition(params.INTAKE_SPEED);
    }

    public void intakeRetract() {
        robot.servoIntakeActuator.setPosition(params.INTAKE_RETRACT_POS);
        robot.servoIntake.setPosition(params.INTAKE_STOP);
    }

    public void intakeReset() {
        robot.servoIntakeActuator.setPosition(params.INTAKE_RESET_POS);
        robot.servoIntake.setPosition(params.INTAKE_STOP);
    }

    public void armHalfwayDown() {
        robot.armMotor.setTargetPosition(params.ARM_HALFWAY_POS);
        robot.armMotor.setPower(params.ARM_IDLE_POWER);
    }

    public void armDown() {
        robot.armMotor.setTargetPosition(params.ARM_DOWN_POS);
        robot.armMotor.setPower(params.ARM_IDLE_POWER);
        params.armDeployed = false;
        params.scoreHigh = false;
    }

    public void armDownAuto() {
        robot.armMotor.setTargetPosition(params.ARM_DOWN_POS);
        robot.armMotor.setPower(params.ARM_IDLE_POWER);
        params.armDeployed = false;
        params.scoreHigh = false;
    }

    public void clawUp() {
        robot.servoClawWrist.setPosition(params.CLAW_WRIST_UP);
    }
    public void clawUpAuto() {
        robot.servoClawWrist.setPosition(params.CLAW_WRIST_UP_AUTO);
    }


    public void clawUpHighScore() {
        robot.servoClawWrist.setPosition(params.CLAW_WRIST_UP_HIGH_SCORE);
    }
    public void clawDown() {
        robot.servoClawWrist.setPosition(params.CLAW_WRIST_DOWN);
    }

//    Thread liftThread = new Thread(() -> {
//        if(robot.secondMotorLift.getCurrent(CurrentUnit.AMPS) > 4) {
//            robot.secondMotorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//            robot.secondMotorLift.setMotorDisable();
//            robot.secondMotorLift.setPower(0);
//        } else {
//            robot.secondMotorLift.setMotorEnable();
//            robot.secondMotorLift.setPower(1);
//        }
//    });

    public int liftPosition(int liftPosition) {
        liftPosition = Range.clip(liftPosition, 0, params.LIFT_MAX_HEIGHT);

        robot.motorLift.setPower(1); // was 1
        robot.motorLift.setTargetPosition(liftPosition);
//        robot.motorLift.setTargetPositionTolerance(1);
//        robot.secondMotorLift.setTargetPositionTolerance(500);

        return liftPosition;
    }

    public void liftPositionNoReturn(int liftPosition) {
        int liftPos = this.liftPosition(liftPosition);
    }

    public void initForAuto(){
        clawRightClose();
        clawLeftClose();
        opMode.sleep(100);
    }

    public void climbUp() {
        robot.climbMotorLeft.setPower(1);
        robot.climbMotorRight.setPower(1);
    }

    public void homeClimb() {
        climbRightHomed = false;
        climbLeftHomed = false;

        new Thread(() -> {
            while (true) {
                if (!climbLeftHomed) {
                    robot.climbMotorLeft.setPower(-.3);
                } else {
                    robot.climbMotorLeft.setPower(0);
                }
                if (robot.climbMotorLeft.getCurrent(CurrentUnit.AMPS) > params.CLIMB_HOME_AMPERAGE) {
                    climbLeftHomed = true;
                }

                if (!climbRightHomed) {
                    robot.climbMotorRight.setPower(-.3);
                } else {
                    robot.climbMotorRight.setPower(0);
                }
                if (robot.climbMotorRight.getCurrent(CurrentUnit.AMPS) > params.CLIMB_HOME_AMPERAGE) {
                    climbRightHomed = true;
                }

                if(climbLeftHomed && climbRightHomed) break;
            }
        }).start();
    }

    public void climbDown() {
        robot.climbMotorLeft.setPower(-1);
        robot.climbMotorRight.setPower(-1);
    }

    public void scorePurplePixel() {
        this.clawLeftClose();
        this.clawRightClose();
        opMode.sleep(100);
        opMode.sleep(1250);
        this.clawRightOpen();
        this.clawLeftOpen();
        opMode.sleep(200);
        this.clawLeftClose();
        this.clawRightClose();
    }

    public void liftReset(){
        liftPosition(params.LIFT_LOW_2_POSITION);
        robot.motorLift.setPower(params.LIFT_POWER_DOWN);
    }

    public void droneFire() {
        robot.servoDrone.setPosition(params.DRONE_FIRE);
    }


    public boolean clawHasPixel(RevColorSensorV3 sensor) {
        if(sensor.getDistance(DistanceUnit.CM) < params.CLAW_SENSE_DISTANCE) {
            return true;
        } else {
            return false;
        }
    }

    public void clawRightLedState(boolean state) {
        robot.rightLedBackRed.setState(!state);
        robot.rightLedBackGreen.setState(state);

        robot.rightLedFrontRed.setState(!state);
        robot.rightLedFrontGreen.setState(state);
    }

    public void clawLeftLedState(boolean state) {
        robot.leftLedBackRed.setMode(DigitalChannel.Mode.OUTPUT);
        robot.leftLedBackRed.setState(!state);
        robot.leftLedBackGreen.setMode(DigitalChannel.Mode.OUTPUT);
        robot.leftLedBackGreen.setState(state);

        robot.leftLedFrontRed.setMode(DigitalChannel.Mode.OUTPUT);
        robot.leftLedFrontRed.setState(!state);
        robot.leftLedFrontGreen.setMode(DigitalChannel.Mode.OUTPUT);
        robot.leftLedFrontGreen.setState(state);
    }

    public boolean getColor(RevColorSensorV3 sensor) {
        if(Math.round(sensor.getNormalizedColors().toColor() / 100) < 0 && Math.round(sensor.getNormalizedColors().toColor() / 100) > -10_100_100) {
            return true;
        } else {
            return false;
        }
    }

//    public void homeArm() {
//        new Thread(() -> {
//            homed = false;
//            while (!homed) {
//                robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                robot.armMotor.setPower(-.35);
//                if (robot.armMotor.getCurrent(CurrentUnit.AMPS) > 3) {
//                    homed = true;
//                }
//            }
//            robot.armMotor.setPower(0);
//            robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }).start();
//    }


    public void armUpMid() {
        robot.armMotor.setTargetPosition(params.ARM_MIDWAY_UP);
        robot.armMotor.setPower(params.ARM_IDLE_POWER);
    }

    public void clawFinger() {
        robot.servoClawWrist.setPosition(params.CLAW_FINGER_POS);
    }

    public void armCustomPos(int pos) {
        robot.armMotor.setPower(params.ARM_IDLE_POWER);
        robot.armMotor.setTargetPosition(pos);
    }

    public void armCustomPosAndPower(int pos, double power) {
        robot.armMotor.setPower(power);
        robot.armMotor.setTargetPosition(pos);
    }

    public void climbLock() {
        robot.servoClimbLockLeft.setPosition(params.LOCK_SERVO_POS);
        robot.servoClimbLockRight.setPosition(1 - params.LOCK_SERVO_POS);
    }

    public void climbUnlock() {
        robot.servoClimbLockLeft.setPosition(params.UNLOCK_SERVO_POS);
        robot.servoClimbLockRight.setPosition(1 - params.UNLOCK_SERVO_POS);
    }

    public void clawOffGround() {
        robot.servoClawWrist.setPosition(params.CLAW_WRIST_MID_POS);
    }
}