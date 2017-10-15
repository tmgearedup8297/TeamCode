package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by govindb on 01/22/17.
 */
public abstract class ShooterTWOBeaconsSpinTurn extends AutonomousVelVort {


    public void activateShooterTWOBeaconsSpinTurn(String colorString) throws InterruptedException {

        shootBall(); // shoot first particle

        moveStraight(0.2, 4.5, true);

        // Extend both servos at the beginning so second  particle can drop into the shooter area
        redServo.setPosition(RED_SERVO_EXTENDED);
        blueServo.setPosition(BLUE_SERVO_EXTENDED);

        sleep(500);


        runIntake(1); //run intake for 1 second to drop second particle into the shooter

        shootBall(); // shoot second particle

        shootBall(); // shoot second particle

        closeBallShooter(330); // it moves the shooter to bring it to the correct position

        // Retract both servos now both particles have been shot into the center vortex
        redServo.setPosition(RED_SERVO_RETRACTED);
        blueServo.setPosition(BLUE_SERVO_RETRACTED);


        sleep(400);

        moveStraight(0.25, 23, true); //moves towards beacon

        sleep(500);

        if(colorString.equalsIgnoreCase("blue")) { //if blue alliance is selected...

            spinTurn(24, 0.25, moveDirection.RIGHT);

        } else if (colorString.equalsIgnoreCase("red")){ //if red alliance is selected...

            spinTurn(-24, 0.25, moveDirection.LEFT);

        }

        moveStraight(0.4, 90, true);

        sleep(500);

        alignRobot(0.2);

        sleep(400);

        findWhiteLine((float) 0.14, 20); //aligns with beacon

        sleep(700);

        alignRobot(0.15);

        /*
        current_orientation = dof.getAngularOrientation();
        realOrientation = (int) (current_orientation.firstAngle - angle_0);

        telemetry.addData("GYRO STOP VALUE", -realOrientation);
        telemetry.update();

        if((formatDOFValue(realOrientation)) < DOF_RED_THRESHOLD){
            spinTurn(DOF_RED_THRESHOLD, 0.15, moveDirection.RIGHT);
        } else if ((formatDOFValue(realOrientation)) > DOF_BLUE_THRESHOLD){
            spinTurn(DOF_BLUE_THRESHOLD, 0.15, moveDirection.LEFT);

        }
        */

        if(colorString.equalsIgnoreCase("blue")) { //if blue alliance is selected...

            driveUntilUltrasonic((float) 0.65, (float)STRAFE_SHORT, AutonomousVelVort.moveDirection.RIGHT, BLUEsensorRange); //moves closer beacon by strafing to the right based on range sensor reading

        } else if (colorString.equalsIgnoreCase("red")){ //if red alliance is selected...

            driveUntilUltrasonic((float) 0.65, (float)STRAFE_SHORT, AutonomousVelVort.moveDirection.LEFT, REDsensorRange); //moves closer to beacon by strafing to the left based on range sensor reading

        }

        sleep(400);


        if(colorString.equalsIgnoreCase("blue")) { //if blue alliance is selected...


            if ((BLUEfrontColor.red() > BLUEfrontColor.blue()) || (BLUEbackColor.blue() > BLUEbackColor.red())) { //if left button is red and right button is blue

                telemetry.addData("PUSH FAR RIGHT BUTTON", "");

                moveStraight(0.20, 3.5, false); //move back so so pusher is aligned

                activateBeacon(blueServo);

            } else if ((BLUEfrontColor.red() < BLUEfrontColor.blue()) || (BLUEbackColor.red() > BLUEbackColor.blue())) { //if left button is blue and right button is red

                telemetry.addData("PUSH FAR LEFT BUTTON", "");

                moveStraight(0.20, 2, true); //move forward so pusher is aligned

                activateBeacon(blueServo);


            } else if (BLUEfrontColor.blue() == BLUEfrontColor.red()) { //if there are error readings
                telemetry.addData("NOT READING ANYTHING", "");
                telemetry.addData("DO NOT PUSH ANY BUTTON", "");

            } else if (BLUEbackColor.blue() == BLUEbackColor.red()) { //if there are error readings
                telemetry.addData("NOT READING ANYTHING", "");
                telemetry.addData("DO NOT PUSH ANY BUTTON", "");

            }

            moveByTime(0.85, moveDirection.LEFT, 150);


        } else if (colorString.equalsIgnoreCase("red")){ // if red alliance is selected


            if ((REDfrontColor.red() > REDfrontColor.blue()) || (REDbackColor.blue() > REDbackColor.red())) { // OR condition added

                telemetry.addData("PUSH FAR RIGHT BUTTON", "");

                moveStraight(0.20, 1, true); //move forward so so pusher is aligned

                activateBeacon(redServo);


            } else if ((REDfrontColor.red() < REDfrontColor.blue()) || (REDbackColor.red() > REDbackColor.blue())) { // OR condition added

                telemetry.addData("PUSH FAR LEFT BUTTON", "");

                moveStraight(0.20, 4.5, false); //move back so robot is in center

                activateBeacon(redServo);

            } else {
                telemetry.addData("NOT READING ANYTHING", "");
                telemetry.addData("DO NOT PUSH ANY BUTTON", "");
            }

            moveByTime(0.85, moveDirection.RIGHT, 150);

        }

        sleep(200);

        //alignRobot(0.15);


/*
        current_orientation = dof.getAngularOrientation();
        realOrientation = (int) (current_orientation.firstAngle - angle_0);

        telemetry.addData("GYRO STOP VALUE", -realOrientation);
        telemetry.update();

        if((formatDOFValue(realOrientation)) < DOF_RED_THRESHOLD){
            spinTurn(DOF_RED_THRESHOLD, 0.15, moveDirection.RIGHT);
        } else if ((formatDOFValue(realOrientation)) > DOF_BLUE_THRESHOLD){
            spinTurn(DOF_BLUE_THRESHOLD, 0.15, moveDirection.LEFT);

        }
   */

        sleep(200);

        telemetry.update();

        moveStraight(0.3, 40, false); //moves towards second beacon

        sleep(600); //Do not change

        findWhiteLine((float) -0.14, 14); //aligns with beacon

        sleep(500);

        alignRobot(0.15);


/*
        current_orientation = dof.getAngularOrientation();
        realOrientation = (int) (current_orientation.firstAngle - angle_0);

        telemetry.addData("GYRO STOP VALUE", -realOrientation);
        telemetry.update();

        if((formatDOFValue(realOrientation)) < DOF_RED_THRESHOLD){
            spinTurn(DOF_RED_THRESHOLD, 0.15, moveDirection.RIGHT);
        } else if ((formatDOFValue(realOrientation)) > DOF_BLUE_THRESHOLD){
            spinTurn(DOF_BLUE_THRESHOLD, 0.15, moveDirection.LEFT);

        }
*/
        sleep(200);


        if(colorString.equalsIgnoreCase("blue")) {

            driveUntilUltrasonic((float) 0.6, (float)STRAFE_SECOND, AutonomousVelVort.moveDirection.RIGHT, BLUEsensorRange);

        } else if (colorString.equalsIgnoreCase("red")){

            driveUntilUltrasonic((float) 0.6, (float)STRAFE_SECOND, AutonomousVelVort.moveDirection.LEFT, REDsensorRange);

        }

        sleep (500);

        if(colorString.equalsIgnoreCase("blue")) {

            if ((BLUEfrontColor.red() > BLUEfrontColor.blue()) || (BLUEbackColor.blue() > BLUEbackColor.red())) {

                telemetry.addData("PUSH FAR RIGHT BUTTON", "");

                moveStraight(0.15, 2, false); //move back so so pusher is aligned

                activateBeacon(blueServo);

            } else if ((BLUEfrontColor.red() < BLUEfrontColor.blue()) || (BLUEbackColor.red() > BLUEbackColor.blue())) {

                telemetry.addData("PUSH FAR LEFT BUTTON", "");

                moveStraight(0.15, 3.5, true); //move forward so so pusher is aligned

                activateBeacon(blueServo);

            } else if (BLUEfrontColor.blue() == BLUEfrontColor.red()) {
                telemetry.addData("NOT READING ANYTHING", "");
                telemetry.addData("DO NOT PUSH ANY BUTTON", "");

            } else if (BLUEbackColor.blue() == BLUEbackColor.red()) {
                telemetry.addData("NOT READING ANYTHING", "");
                telemetry.addData("DO NOT PUSH ANY BUTTON", "");

            }

        } else if (colorString.equalsIgnoreCase("red")){


            if ((REDfrontColor.red() > REDfrontColor.blue()) || (REDbackColor.blue() > REDbackColor.red())) { //OR condition

                telemetry.addData("PUSH FAR RIGHT BUTTON", "");

                moveStraight(0.20, 3, true); //move forward so pusher is aligned

                activateBeacon(redServo);


            } else if ((REDfrontColor.red() < REDfrontColor.blue())  || (REDbackColor.red() > REDbackColor.blue())) { // OR condition

                telemetry.addData("PUSH FAR LEFT BUTTON", "");

                moveStraight(0.20, 2, false); //move back so so pusher is aligned

                activateBeacon(redServo);

            } else if (REDfrontColor.blue() == REDfrontColor.red()) {
                telemetry.addData("NOT READING ANYTHING", "");
                telemetry.addData("DO NOT PUSH ANY BUTTON", "");

            } else if (REDbackColor.blue() == REDbackColor.red()) {
                telemetry.addData("NOT READING ANYTHING", "");
                telemetry.addData("DO NOT PUSH ANY BUTTON", "");

            }

        }

        if(colorString.equalsIgnoreCase("red")){
            moveByTime(0.9, moveDirection.RIGHT, 500);

        } else{
            moveByTime(0.9, moveDirection.LEFT, 500);

        }

        moveStraight(0.5, 45, false); //moves towards second beacon

        moveStraight(0.15, 3, false); //moves towards second beacon


    }


}
