package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by govindb on 1/7/17.
 */
public abstract class ShooterDefensiveCapBall extends AutonomousVelVort {

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

    public void activateShooterDefensiveCapBall(String colorString) throws InterruptedException{

        /**
         * SECTION ONE: ATTEMPTS TWO PARTICLE SHOTS IN CENTER VORTEX
         */

        sleep(2000);

        moveStraight(0.2, 12, true); //drive forward so that shooter clears the wall

        sleep(200);

        //strafe left/right into firing location
        if(colorString.equalsIgnoreCase("red")){
            spinTurn(-27, 0.25, moveDirection.LEFT);
        } else {
            spinTurn(27, 0.25, moveDirection.RIGHT);
        }

        sleep(700);

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

        alignRobot(0.2);

        sleep(500);

        moveStraight(0.5, 78, true); //move towards center vortex


    }

}
