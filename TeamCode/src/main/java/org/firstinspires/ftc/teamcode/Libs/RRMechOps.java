package org.firstinspires.ftc.teamcode.Libs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.Params;
import org.firstinspires.ftc.teamcode.Hardware.RRHWProfile;

@Config
public class RRMechOps {

    private RRHWProfile robot;
    public LinearOpMode opMode;
    public Params params;
    public double RRPower;
    public double RFPower;
    public double LRPower;
    public double LFPower;

    /*
     * Constructor method
     */
    public RRMechOps(RRHWProfile myRobot, LinearOpMode myOpMode, Params myParams){
        robot = myRobot;
        opMode = myOpMode;
        params = myParams;

    }   // close RRMechOps constructor Method

    public void liftPos(int liftPos) {
        liftPos = Range.clip(liftPos, 0, 1290);

        robot.motorLift.setPower(1);
        robot.motorLift.setTargetPosition(liftPos);
    }

    public void bucketReset() {
        this.clawleftclose();
        this.clawRightClose();
        armIdle();
        robot.servoBucket.setPosition(params.BUCKET_RESET);
    }


    public void bucketScore() {
        this.clawleftclose();
        this.clawRightClose();
        armIdle();
        robot.servoBucket.setPosition(params.BUCKET_SCORE);
    }

    public void bucketScoreLiftUp() {
        armIdle();
        robot.servoBucket.setPosition(params.BUCKET_SCORE);
        opMode.sleep(500);
        this.liftPos(500);
        opMode.sleep(1000);
        this.liftPos(0);
    }
    public void bucketLineUp() {
        this.clawleftclose();
        this.clawRightClose();
        armIdle();
        robot.servoBucket.setPosition(params.BUCKET_LINE_UP);
    }

    public void motorsHalt() {
        robot.motorLF.setPower(0);
        robot.motorLR.setPower(0);
        robot.motorRF.setPower(0);
        robot.motorRR.setPower(0);
    }

    public double getZAngle(){
        return (robot.imu.getAbsoluteHeading());
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

public void loadPixels(){
        liftReset();
        bucketReset();
        armReset();
        wristPosition(params.WRIST_LOAD_PIXELS);
        opMode.sleep(100);
        clawRightOpen();
        clawLeftOpen();
        opMode.sleep(50);
        wristPosition(params.WRIST_EXTEND);
        armExtend();
    }

    public void wristPosition(double position) {
        robot.servoWrist.setPosition(position);
    }

    public void clawLeftOpen(){
        robot.servoClawLeft.setPosition(params.CLAW_LEFT_OPEN);
    }

    public void clawleftclose(){
        robot.servoClawLeft.setPosition(params.CLAW_LEFT_CLOSE);
    }

    public void clawRightOpen(){
        robot.servoClawRight.setPosition(params.CLAW_RIGHT_OPEN);
    }

    public void clawRightClose(){
        robot.servoClawRight.setPosition(params.CLAW_RIGHT_CLOSE);
    }

    public void armExtend(){
        robot.servoArmLeft.setPosition(params.ARM_LEFT_EXTEND);
        robot.servoArmRight.setPosition(params.ARM_RIGHT_EXTEND);
    }

    public void slidesExtend(){
        robot.servoSlideLeft.setPosition(params.SLIDE_LEFT_EXTEND);
        robot.servoSlideRight.setPosition(params.SLIDE_RIGHT_EXTEND);
    }

    public void slidesReset(){
        robot.servoSlideLeft.setPosition(params.SLIDE_LEFT_RESET);
        robot.servoSlideRight.setPosition(params.SLIDE_RIGHT_RESET);
    }

    public void armIdle(){
        robot.servoArmLeft.setPosition(params.ARM_LEFT_IDLE);
        robot.servoArmRight.setPosition(params.ARM_RIGHT_IDLE);
        this.wristPosition(params.WRIST_EXTEND);
    }
    public void armLowIdle(){
        robot.servoArmLeft.setPosition(params.ARM_LEFT_EXTEND_LOW_IDLE);
        robot.servoArmRight.setPosition(params.ARM_RIGHT_EXTEND_LOW_IDLE);
    }

    public void armReset(){
        robot.servoArmLeft.setPosition(params.ARM_LEFT_RESET);
        robot.servoArmRight.setPosition(params.ARM_RIGHT_RESET);
    }
    public void armExtendBlock(){
        robot.servoArmLeft.setPosition(params.ARM_LEFT_EXTEND);
        robot.servoArmRight.setPosition(params.ARM_RIGHT_EXTEND);
    }

    public void droneLoad() {
        robot.servoDrone.setPosition(params.DRONE_LOAD);
    }


    public void liftPosition(int liftPosition) {
        liftPosition = Range.clip(liftPosition, 0, params.LIFT_MAX_HEIGHT);

        robot.motorLift.setPower(params.LIFT_POWER);
        robot.motorLift.setTargetPosition(liftPosition);
    }

    public void scorePurplePixel() {
        this.clawleftclose();
        this.clawRightClose();
        opMode.sleep(100);
        this.slidesReset();
        this.armExtendBlock();
        this.wristPosition(params.WRIST_EXTEND);
        opMode.sleep(1250);
        this.clawLeftOpen();
        opMode.sleep(100);
        this.armIdle();
        this.clawleftclose();
        this.clawRightClose();
    }

    public void extendForPurplePixel() {
        this.clawleftclose();
        this.clawRightClose();
        opMode.sleep(100);
        this.slidesReset();
        this.armExtendBlock();
        this.wristPosition(params.WRIST_EXTEND);
    }

    public void scoreLowPurplePixel(){
        this.clawLeftOpen();
        opMode.sleep(100);
        this.armIdle();
        this.clawleftclose();
        this.clawRightClose();
    }

    public void liftReset(){
        armIdle();
        bucketReset();
        robot.motorLift.setPower(params.LIFT_POWER_DOWN);
        robot.motorLift.setTargetPosition(params.LIFT_RESET);
    }

    public void fingerOut() {
        robot.fingerServo.setPosition(params.FINGER_OUT);
    }
    public void fingerIn() {
        robot.fingerServo.setPosition(params.FINGER_IN);
    }

    public void droneFire() {
        robot.servoDrone.setPosition(params.DRONE_FIRE);
    }

    public void dronePosition(int pos) {
        robot.droneActuator.setPower(params.DRONE_ACTUATOR_POWER);
        robot.droneActuator.setTargetPosition(pos);
    }
    public void slowBucket() {
        for (double i = params.BUCKET_RESET; i >= params.BUCKET_AUTO_SCORE; i -= 0.0035) {
            robot.servoBucket.setPosition(i);
            opMode.sleep(1);
        }
    }

    public void autoScore() {
//        liftPosition(params.LIFT_AUTO_SCORE);
        opMode.sleep(100);
        slowBucket();
        opMode.sleep(250);
        liftPosition(robot.motorLift.getTargetPosition() + params.LIFT_AUTO_SCORE);
        opMode.sleep(750);
        bucketReset();
//        liftReset();
    }
    public void autoScoreLiftUp() {
        liftPosition(params.LIFT_AUTO_SCORE);
        opMode.sleep(1000);
        autoScore();
    }

    public boolean getColor(RevColorSensorV3 sensor) {
        if(Math.round(sensor.getNormalizedColors().toColor() / 100) < 0 && Math.round(sensor.getNormalizedColors().toColor() / 100) > -10_100_100) {
            return true;
        } else {
            return false;
        }
    }

    public void slidesCustomExtension(double slidesLeftExtend, double slidesRightExtend) {
        robot.servoSlideLeft.setPosition(slidesLeftExtend);
        robot.servoSlideRight.setPosition(slidesRightExtend);
    }
}   // close the RRMechOps class