package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class AutonomousRelicRecovery extends MasterDeviceClass {
    //TODO: find out what these variables mean
    static final double LEFT_FAST_STRAFE_MULTIPLIER = 1.07;

    static final double RIGHT_FAST_STRAFE_MULTIPLIER = 1.05;

    static final double LEFT_SLOW_STRAFE_MULTIPLIER = 1.15;

    static final double RIGHT_SLOW_STRAFE_MULTIPLIER = 1.16;

    static final double LEFT_MOVE_STRAIGHT_MULTIPLIER = 1.08;

    static final float STRAFE_OUT_DISTANCE_ULTRASONIC = 5;

    public void stopDriveMotors() {
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);

        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
    }

    public void moveStraight(double nPower, double distance, boolean direction) throws InterruptedException {

        nPower = (getPowerMultiplier(nPower)* nPower);

        long rightEncoderSP, leftEncoderSP;

        float totalTicks = (rotationsPerInch * (float) distance * ticksRotation) * (float) sprocketReductionRatio; //Calculate the total ticks to move.

        rightEncoderSP = -motorRightFront.getCurrentPosition(); // read the right motor encoder before starting in the loop
        leftEncoderSP = -motorLeftBack.getCurrentPosition();  // read the left motor encoder before starting in the loop


        //While the current distance traveled is less than the target distance, keep driving. Afterwards, stop all motors.
        if (direction) {

            while (((-motorRightFront.getCurrentPosition() < (rightEncoderSP + totalTicks)) && (-motorLeftBack.getCurrentPosition() < (leftEncoderSP + totalTicks))) && opModeIsActive()) {

                // set power to motors
                motorRightFront.setPower(nPower);
                motorRightBack.setPower(nPower);
                motorLeftFront.setPower(nPower);
                motorLeftBack.setPower(nPower);
            }


        } else {
            while (((-motorRightFront.getCurrentPosition() > (rightEncoderSP - totalTicks)) && (-motorLeftBack.getCurrentPosition() > (leftEncoderSP - totalTicks))) && opModeIsActive()) {

                motorRightFront.setPower(-nPower); //set the power
                motorRightBack.setPower(-nPower);
                motorLeftFront.setPower(-nPower);
                motorLeftBack.setPower(-nPower);
            }

        }

        stopDriveMotors();
    }

    public int formatDOFValue(int headingDof) {

        //Reverse the values found from the adafruit 9DOF sensor
        return -headingDof;
    }

    public void spinTurnUsingDOF(int degrees, double power, moveDirection direction) throws InterruptedException {

        Orientation current_orientation;

        power = (getPowerMultiplier(power)* power);


        // get the first value when you enter the method
        current_orientation = dof.getAngularOrientation();
        realOrientation = (int) (current_orientation.firstAngle - angle_0);

        // LEFT turn values always have to come in with -ve DEGREES
        if (direction == moveDirection.RIGHT) {
            while ((formatDOFValue(realOrientation) < degrees) && opModeIsActive()) {

                motorLeftBack.setPower(power);
                motorLeftFront.setPower(power);
                motorRightFront.setPower(-power);
                motorRightBack.setPower(-power);

                // refresh the  value inside the while loop

                current_orientation = dof.getAngularOrientation();
                realOrientation = (int) (current_orientation.firstAngle - angle_0);

            }
            stopDriveMotors();
            telemetry.addData("DOF gyro value WHILE RIGHT", realOrientation);
            telemetry.update();

            // RIGHT turn values always have to come in with +ve DEGREES


        } else if (direction == moveDirection.LEFT) {

            while ((formatDOFValue(realOrientation) > degrees) && opModeIsActive()) {
                motorLeftBack.setPower(-power);
                motorLeftFront.setPower(-power);
                motorRightFront.setPower(power);
                motorRightBack.setPower(power);

                // refresh the  value inside the while loop

                current_orientation = dof.getAngularOrientation();
                realOrientation = (int) (current_orientation.firstAngle - angle_0);

            }
            stopDriveMotors();

            telemetry.addData("DOF gyro value WHILE LEFT", realOrientation);
            telemetry.update();
        }

        //Stop the motors once both if statements are completed.
        stopDriveMotors();

    }

    public void moveByTime(double power, moveDirection dir, long timeMill) throws InterruptedException {

        //Move by time in a given direction

        power = (getPowerMultiplier(power)* power);


        if (dir == moveDirection.FORWARD) {
            motorLeftFront.setPower(power);
            motorLeftBack.setPower(power);
            motorRightFront.setPower(power);
            motorRightBack.setPower(power);
        } else if (dir == moveDirection.BACK) {
            motorLeftFront.setPower(-power);
            motorLeftBack.setPower(-power);
            motorRightFront.setPower(-power);
            motorRightBack.setPower(-power);
        } else if (dir == moveDirection.RIGHT) {
            motorLeftFront.setPower(power);
            motorLeftBack.setPower(-RIGHT_FAST_STRAFE_MULTIPLIER *power); //power increase trying to fix strafe
            motorRightFront.setPower(-power);
            motorRightBack.setPower(RIGHT_FAST_STRAFE_MULTIPLIER* power); //power increase trying to fix strafe
        } else if (dir == moveDirection.LEFT) {
            motorLeftFront.setPower(-power);
            motorLeftBack.setPower(LEFT_FAST_STRAFE_MULTIPLIER*(power)); //power increase trying to fix strafe
            motorRightFront.setPower(power);
            motorRightBack.setPower(-LEFT_FAST_STRAFE_MULTIPLIER*(power)); //power increase trying to fix strafe
        }
        sleep(timeMill);
        this.stopDriveMotors();


    }




    //might not need
    public void driveUntilUltrasonic(double power, float distanceIN, moveDirection direction, ModernRoboticsI2cRangeSensor range) throws InterruptedException {
        power = (getPowerMultiplier(power)* power);


        //While the current distance is greater than the target distance, keep moving
        if (direction == moveDirection.RIGHT) {
            while ((range.getDistance(DistanceUnit.INCH) > distanceIN) && opModeIsActive()) {
                motorRightFront.setPower(-power);
                motorRightBack.setPower(RIGHT_SLOW_STRAFE_MULTIPLIER* power);
                motorLeftFront.setPower(power);
                motorLeftBack.setPower(-RIGHT_SLOW_STRAFE_MULTIPLIER *power);

            }
        } else if (direction == moveDirection.LEFT) {
            while ((range.getDistance(DistanceUnit.INCH) > distanceIN) && opModeIsActive()) {
                motorRightFront.setPower(power);
                motorRightBack.setPower(-LEFT_SLOW_STRAFE_MULTIPLIER * power);
                motorLeftFront.setPower(-power);
                motorLeftBack.setPower(LEFT_SLOW_STRAFE_MULTIPLIER *power);
            }
        }

        //telemetry.addData("ULTRASONIC VALUE:", range.getDistance(DistanceUnit.INCH));
        //telemetry.update();
        stopDriveMotors();

    }

    public void strafeOutUntilUltrasonic(double power, float distanceIN, moveDirection direction, ModernRoboticsI2cRangeSensor range) throws InterruptedException {
        power = (getPowerMultiplier(power)* power);

        //While the current distance is greater than the target distance, keep moving
        if (direction == moveDirection.RIGHT) {
            if((range.getDistance(DistanceUnit.INCH) < distanceIN)){
                while ((range.getDistance(DistanceUnit.INCH) < distanceIN) && opModeIsActive()) {
                    motorRightFront.setPower(-power);
                    motorRightBack.setPower(RIGHT_SLOW_STRAFE_MULTIPLIER* power);
                    motorLeftFront.setPower(power);
                    motorLeftBack.setPower(-RIGHT_SLOW_STRAFE_MULTIPLIER *power);

                }
            }

        } else if (direction == moveDirection.LEFT) {
            if((range.getDistance(DistanceUnit.INCH) < distanceIN)) {
                while ((range.getDistance(DistanceUnit.INCH) < distanceIN) && opModeIsActive()) {
                    motorRightFront.setPower(power);
                    motorRightBack.setPower(-LEFT_SLOW_STRAFE_MULTIPLIER * power);
                    motorLeftFront.setPower(-power);
                    motorLeftBack.setPower(LEFT_SLOW_STRAFE_MULTIPLIER * power);
                }
            }
        }
        stopDriveMotors();

    }



    public void exitOpMode() throws InterruptedException {

        motorRightBack.setPower(0);
        motorRightBack.close();
        motorRightFront.setPower(0);
        motorRightFront.close();
        motorLeftBack.setPower(0);
        motorLeftBack.close();
        motorRightFront.setPower(0);
        motorRightFront.close();


        dof.close();
        navx_device.close();

    }

    public void spinTurnUsingNavX(int degrees, double power, moveDirection direction) throws InterruptedException {

        power = (getPowerMultiplier(power)* power);


        // LEFT turn values always have to come in with -ve DEGREES
        if (direction == moveDirection.RIGHT) {
            while ((navx_device.getYaw() < degrees) && opModeIsActive()) {

                motorLeftBack.setPower(power);
                motorLeftFront.setPower(power);
                motorRightFront.setPower(-power);
                motorRightBack.setPower(-power);

            }
            stopDriveMotors();
            telemetry.addData("NAVX gyro value WHILE RIGHT", realOrientation);
            telemetry.update();

            // RIGHT turn values always have to come in with +ve DEGREES


        } else if (direction == moveDirection.LEFT) {

            while ((navx_device.getYaw() > degrees) && opModeIsActive()) {
                motorLeftBack.setPower(-power);
                motorLeftFront.setPower(-power);
                motorRightFront.setPower(power);
                motorRightBack.setPower(power);
            }
            stopDriveMotors();

            telemetry.addData("NAVX gyro value WHILE LEFT", realOrientation);
            telemetry.update();
        }

        //Stop the motors once both if statements are completed.
        stopDriveMotors();

    }

    public void alignRobot (double power) throws InterruptedException {

        //Read variable set up top to determine which gyro to use
        if (choose_gyro == gyro.NAVX) {

            //If the navX is tilted in one direction, turn in the opposite direction.
            if (navx_device.getYaw() < GYRO_RED_THRESHOLD) {
                spinTurnUsingNavX (GYRO_RED_THRESHOLD, power, moveDirection.RIGHT);
            } else if (navx_device.getYaw() > GYRO_BLUE_THRESHOLD) {
                spinTurnUsingNavX(GYRO_BLUE_THRESHOLD, power, moveDirection.LEFT);

            }
        } else if(choose_gyro == gyro.ADAFRUIT){

            //If the adafruit 9dof is tilted in one direction, turn in the opposite direction.

            Orientation current_orientation;

            current_orientation = dof.getAngularOrientation();
            realOrientation = (int) (current_orientation.firstAngle - angle_0);

            if ((formatDOFValue(realOrientation)) < GYRO_RED_THRESHOLD) {
                spinTurnUsingDOF(GYRO_RED_THRESHOLD, power, moveDirection.RIGHT);
            } else if ((formatDOFValue(realOrientation)) > GYRO_BLUE_THRESHOLD) {
                spinTurnUsingDOF(GYRO_BLUE_THRESHOLD, power, moveDirection.LEFT);

            }

        }
    }

    public void moveStraightGyroCorrection(double nPower, double distance, boolean direction) throws InterruptedException {

        nPower = (getPowerMultiplier(nPower)* nPower);

        long rightEncoderSP, leftEncoderSP;


        float totalTicks = (rotationsPerInch * (float) distance * ticksRotation) * (float) sprocketReductionRatio; //Calculate the total ticks to move.


        rightEncoderSP = -motorRightFront.getCurrentPosition(); // read the right motor encoder before starting in the loop
        leftEncoderSP = -motorLeftBack.getCurrentPosition();  // read the left motor encoder before starting in the loop


        //While the current distance traveled is less than the target distance, keep driving. Afterwards, stop all motors.
        if (direction) {

            while (((-motorRightFront.getCurrentPosition() < (rightEncoderSP + totalTicks)) && (-motorLeftBack.getCurrentPosition() < (leftEncoderSP + totalTicks))) && opModeIsActive()) {

                // set power to motors
                motorRightFront.setPower(nPower);
                motorRightBack.setPower(nPower);
                motorLeftFront.setPower(nPower);
                motorLeftBack.setPower(nPower * LEFT_MOVE_STRAIGHT_MULTIPLIER);


            }


        } else {
            while (((-motorRightFront.getCurrentPosition() > (rightEncoderSP - totalTicks)) && (-motorLeftBack.getCurrentPosition() > (leftEncoderSP - totalTicks))) && opModeIsActive()) {

                motorRightFront.setPower(-nPower); //set the power
                motorRightBack.setPower(-nPower);
                motorLeftFront.setPower(-nPower);
                motorLeftBack.setPower(-LEFT_MOVE_STRAIGHT_MULTIPLIER * nPower);

            }

        }

        stopDriveMotors();
    }

    public void spinTurn(int degrees, double power, moveDirection direction) throws InterruptedException {

        //Use the appropriate turn method from above based on the variable from up top that determines the gyro to use
        if(choose_gyro == gyro.NAVX){

            this.spinTurnUsingNavX(degrees, power, direction);

        } else{

            this.spinTurnUsingDOF(degrees, power, direction);
        }


    }

    public void grabGlyph(){
        left.setPosition(0.6);
        right.setPosition(0.6);
    }
    public void letGoOfGlyph(){
        left.setPosition(0);
        right.setPosition(0);
    }
    public void dropJewelTool(){
        jewelTool.setPosition(0.75);
    }
    public void pickUpJewelTool(){
        jewelTool.setPosition(0);
    }

}
