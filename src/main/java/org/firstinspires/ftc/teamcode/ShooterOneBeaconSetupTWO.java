package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public abstract class ShooterOneBeaconSetupTWO extends AutonomousVelVort {

    /**
     * This program attempts to shoot two particles at the start of the Autonomous period,
     * activate two corresponding alliance beacons, and hit the cap ball/park on the center
     * vortex to and wait for the Driver Controlled Period to commence. An overall view of
     * the program:
     * 1. Without moving from starting location, fire first particle
     * 2. Drive forward, fire second particle
     * 3. Strafe left/right and align with first beacon
     * 4. Activate first beacon
     * 5. Drive to second beacon
     * 6. Align with second beacon
     * 7. Activate second beacon
     * 8. Strafe away from beacon and turn
     * 9. Drive to center vortex and hit the cap ball
     *
     * In addition, the program utilizes multiple sensors. These include sensors such as:
     * 1. Modern Robotics Color Sensor (x5)
     * 2. Modern Robotics Range Sensor (x2)
     * 3. REV motor encoders (x2)
     * 4. NavX Micro Gyro Sensor (x1)
     * 5. Adafruit 9DOF Absolute Orientation Sensor (x1)
     */

    public void activateShooterOneBeaconSetupTWO(String colorString) throws InterruptedException {

        /**
         * SECTION ONE: ATTEMPTS TWO PARTICLE SHOTS IN CENTER VORTEX
         */

        shootBall(); // move the shooter by time to shoot first particle

        moveStraight(0.2, 4, true); // drive forward so that shooter clears the wall

        // extend both servos at the beginning so second  particle can drop into the shooter area
        redServo.setPosition(RED_SERVO_EXTENDED);
        blueServo.setPosition(BLUE_SERVO_EXTENDED);
        sleep(500);

        runIntake(1); //run intake for 1 second to drop second particle into the shooter

        shootBall(); // move the shooter to the correct position before firing

        shootBall(); // move the shooter by time to shoot the second particle

        closeBallShooter(300); // close the shooter so that it is out of the way

        // Retract both servos now both particles have been shot into the center vortex
        redServo.setPosition(RED_SERVO_RETRACTED);
        blueServo.setPosition(BLUE_SERVO_RETRACTED);
        sleep(100);

        /**
         * SECTION TWO: ATTEMPTS NEAR BEACON
         */

        moveStraight(0.3, 32, true); //moves 30 in towards center vortex
        sleep(500);

        //Strafe left/right for red/blue for 1.8 seconds so the robot is in the general area of the ramp
        if(colorString.equalsIgnoreCase("red")){
            moveByTime(0.9, moveDirection.LEFT, 1800);
        } else {
            moveByTime(0.9, moveDirection.RIGHT, 1800);
        }
        sleep(200);

        moveStraight(0.4, 7, true); //drive forward so the robot is closer to the white line
        sleep(200);

        //Strafe closer to the wall with the ultrasonic sensor

        if(colorString.equalsIgnoreCase("blue")) {

            driveUntilUltrasonic((float) 0.55, (float)STRAFE_LONG, AutonomousVelVort.moveDirection.RIGHT, BLUEsensorRange);

        } else if (colorString.equalsIgnoreCase("red")){

            driveUntilUltrasonic((float) 0.55, (float)STRAFE_LONG, AutonomousVelVort.moveDirection.LEFT, REDsensorRange);

        }
        sleep(200);

        alignRobot(0.17); //use the gyro to align the robot back to absolute zero
        sleep(200);

        findWhiteLine((float) 0.13, 14); //move forward until the robot hits the white line

        moveStraight(0.12, 2, false); //drive backward to correct for the momentum that carries the robot beyond the white line

        sleep(200);

        //strafe even closer at a lower power using the ultrasonic sensor, to about 4 inches away

        if(colorString.equalsIgnoreCase("blue")) {

            driveUntilUltrasonic((float) 0.38, (float)STRAFE_SHORT, AutonomousVelVort.moveDirection.RIGHT, BLUEsensorRange); //blue has to be different

        } else if (colorString.equalsIgnoreCase("red")){

            driveUntilUltrasonic((float) 0.38, (float)STRAFE_SHORT, AutonomousVelVort.moveDirection.LEFT, REDsensorRange);

        }
        sleep (200);
        alignRobot(0.17); //align the robot back to absolute zero again to correct for error in robot movement
        sleep(500);

        //Make decision on which button to press based on color sensor, and push the correct button
        if(colorString.equalsIgnoreCase("blue")) {

            if ((BLUEfrontColor.red() > BLUEfrontColor.blue()) || (BLUEbackColor.blue() > BLUEbackColor.red())) {

                telemetry.addData("PUSH FAR RIGHT BUTTON", "");

                moveStraight(0.15, 1.75, false); //move back so so pusher is aligned

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
