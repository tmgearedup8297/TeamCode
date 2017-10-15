package org.firstinspires.ftc.teamcode;

/**
 * Created by govindb on 11/26/16.
 */
public abstract class ShooterCapBall extends AutonomousVelVort {

    public void activateShooterCapBall(String colorString) throws InterruptedException {


        sleep(8000); //adjust this one based on match settings!

        moveStraight(0.25, 22, true);

        sleep(2000);

        shootBall(); // shoot first particle


        // Extend both servos at the beginning so second  particle can drop into the shooter area
        redServo.setPosition(RED_SERVO_EXTENDED);
        blueServo.setPosition(BLUE_SERVO_EXTENDED);
        sleep(1000);


        runIntake(1); //run intake for 1 second to drop second particle into the shooter

        shootBall(); // move shooter to correct position to shoot

        shootBall(); // shoot second particle

        closeBallShooter(300); // it moves the shooter to bring it to the closed position

        // Retract both servos now both particles have been shot into the center vortex
        redServo.setPosition(RED_SERVO_RETRACTED);
        blueServo.setPosition(BLUE_SERVO_RETRACTED);

        moveStraight(0.60, 42, true); //moves ball toward center vortex
        sleep(2000);

        moveStraight(0.20, 10, true); //moves forwards and parks on center vortex


    }

}
