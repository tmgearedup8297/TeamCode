package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by govindb on 1/7/17.
 */
public abstract class ShooterOneBeacon extends AutonomousVelVort {

    /**
     * This program attempts to shoot two particles at the start of the Autonomous period,
     * activate one corresponding alliance beacon, and hit the cap ball/park on the center
     * vortex to and wait for the Driver Controlled Period to commence. An overall view of
     * the program:
     * 1. Drive forward
     * 2. Srafe left/right, fire two particles
     * 3. Drive forward, then strafe left/right towards ramp
     * 4. Align with near beacon
     * 5. Activate near beacon
     * 6. Strafe away from near beacon
     * 7. Strafe into cap ball, then drive backwards so ball is released
     * 8. Turn around
     * 9. Drive backwards and park
     *
     * In addition, the program utilizes multiple sensors. These include sensors such as:
     * 1. Modern Robotics Color Sensor (x5)
     * 2. Modern Robotics Range Sensor (x2)
     * 3. REV motor encoders (x2)
     * 4. NavX Micro Gyro Sensor (x1)
     * 5. Adafruit 9DOF Absolute Orientation Sensor (x1)
     */

    //TODO: PROGRAM IS UNFINISHED, TEST MORE


    public void activateShooterOneBeacon(String colorString) throws InterruptedException{

        /**
         * SECTION ONE: ATTEMPTS TWO PARTICLE SHOTS IN CENTER VORTEX
         */
        moveStraight(0.2, 4.5, true); //drive forward so that shooter clears the wall

        sleep(200);

        //strafe left/right into firing location
        if(colorString.equalsIgnoreCase("red")){
            moveByTime(0.9, moveDirection.LEFT, 1100);
        } else {
            moveByTime(0.9, moveDirection.RIGHT, 1100);
        }

        sleep(500);

        alignRobot(0.16); //align back to absolute zero before shooting

        sleep(400);

        shootBall(); //move shooter by time to shoot first particle

        // Extend both servos at the beginning so second  particle can drop into the shooter area
        redServo.setPosition(RED_SERVO_EXTENDED);
        blueServo.setPosition(BLUE_SERVO_EXTENDED);
        sleep(500);


        runIntake(1); //run intake for 1 second to drop second particle into the shooter

        shootBall(); // move shooter into correct position before firing second particle

        shootBall(); // shoot second particle

        closeBallShooter(330); // move the shooter by time to bring it to the closed position

        // Retract both servos now both particles have been shot into the center vortex
        redServo.setPosition(RED_SERVO_RETRACTED);
        blueServo.setPosition(BLUE_SERVO_RETRACTED);
        sleep(400);

        /**
         * SECTION TWO: ATTEMPTS NEAR BEACON
         */

        moveStraight(0.25, 32, true); //move towards center vortex
        sleep(400);

        //Strafe left/right so that robot is in the general area of the ramp

        if(colorString.equalsIgnoreCase("red")){
            moveByTime(0.9, moveDirection.LEFT, 1800);
        } else {
            moveByTime(0.9, moveDirection.RIGHT, 1800);
        }
        sleep(300);

        moveStraight(0.4, 7.5, true); //drive forward near the white line
        sleep(300);

        //Strafe in using the ultrasonic sensor

        if(colorString.equalsIgnoreCase("blue")) {

            driveUntilUltrasonic((float) 0.5, (float)STRAFE_LONG, AutonomousVelVort.moveDirection.RIGHT, BLUEsensorRange); //moves closer beacon by strafing to the right based on range sensor reading

        } else if (colorString.equalsIgnoreCase("red")){ //if red alliance is selected...

            driveUntilUltrasonic((float) 0.5, (float)STRAFE_LONG, AutonomousVelVort.moveDirection.LEFT, REDsensorRange); //moves closer to beacon by strafing to the left based on range sensor reading

        }
        sleep(300);

        alignRobot(0.16); //align the robot back to absolute zero
        sleep(300);

        findWhiteLine((float) 0.12, 14); //drive to white line

        moveStraight(0.12, 2, false); //drive backwards to correct for the robot's momentum that carried it past the white line

        sleep(300);

        //Strafe further in at a lower power using the ultrasonic sensor

        if(colorString.equalsIgnoreCase("blue")) {

            driveUntilUltrasonic((float) 0.38, (float)STRAFE_SHORT, AutonomousVelVort.moveDirection.RIGHT, BLUEsensorRange); //blue has to be different

        } else if (colorString.equalsIgnoreCase("red")){

            driveUntilUltrasonic((float) 0.38, (float)STRAFE_SHORT, AutonomousVelVort.moveDirection.LEFT, REDsensorRange);

        }
        sleep (200);
        alignRobot(0.16); //align the robot back to absolute zero
        sleep(500);

        //using color sensors, read the color of the beacon, choose the correct side to press, and press the button
        if(colorString.equalsIgnoreCase("blue")) {

            if ((BLUEfrontColor.red() > BLUEfrontColor.blue()) || (BLUEbackColor.blue() > BLUEbackColor.red())) {

                telemetry.addData("PUSH FAR RIGHT BUTTON", "");

                moveStraight(0.15, 1.5, false); //move back so so pusher is aligned

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

                moveStraight(0.20, 2, true); //move forward so pusher is aligned

                activateBeacon(redServo);


            } else if ((REDfrontColor.red() < REDfrontColor.blue())  || (REDbackColor.red() > REDbackColor.blue())) { // OR condition

                telemetry.addData("PUSH FAR LEFT BUTTON", "");

                moveStraight(0.20, 2.5, false); //move back so so pusher is aligned

                activateBeacon(redServo);

            } else if (REDfrontColor.blue() == REDfrontColor.red()) {
                telemetry.addData("NOT READING ANYTHING", "");
                telemetry.addData("DO NOT PUSH ANY BUTTON", "");

            } else if (REDbackColor.blue() == REDbackColor.red()) {
                telemetry.addData("NOT READING ANYTHING", "");
                telemetry.addData("DO NOT PUSH ANY BUTTON", "");

            }

        }
        sleep(100);
        alignRobot(0.16);//align back to absolute zero
        sleep(300);

        /**
         * SECTION THREE: ATTEMPTS CAP BALL AND PARK
         */

        // Strafe towards cap ball
        if(colorString.equalsIgnoreCase("red")) {

            moveByTime( 0.9, AutonomousVelVort.moveDirection.RIGHT, 1900);

        } else if (colorString.equalsIgnoreCase("blue")){

            moveByTime( 0.9, AutonomousVelVort.moveDirection.LEFT, 1900);

        }
        moveStraight(0.6, 6, false); // dislodges the cap ball
        sleep(1500); //do not change, need to wait till the ball rolls away

        //turn around so that the back of the robot faces the center vortex
        if(colorString.equalsIgnoreCase("red")) {
            spinTurn(-120, (float) 0.25, moveDirection.LEFT); ///turns left 120 degrees

        } else{
            spinTurn(120, (float) 0.25, moveDirection.RIGHT); ///turns right 120 degrees

        }

        moveStraight(0.3, 26, false); // partially parks on the center vortex platform by driving backwards

    }

}
