package org.firstinspires.ftc.teamcode;

/**
 * Created by govindb on 1/7/17.
 */
public abstract class ShooterDefensive extends AutonomousVelVort {


    public void activateShooterDefensive(String colorString) throws InterruptedException{

        moveStraight(0.5, 11, true);

        sleep(300);

        if(colorString.equalsIgnoreCase("red")) {
            spinTurn((-26), (float) 0.25, AutonomousVelVort.moveDirection.LEFT); //turns left towards beacon based on absolute orientation reading
        } else{
            spinTurn((26), (float) 0.25, AutonomousVelVort.moveDirection.RIGHT); //turns left towards beacon based on absolute orientation reading

        }
        sleep(1000);

        moveStraight(0.3, 12, true);

        sleep(2500);

        shootBall();

        redServo.setPosition(RED_SERVO_EXTENDED);
        blueServo.setPosition(BLUE_SERVO_EXTENDED);
        sleep(1250);

        runIntake(1); //run intake for 1 second to drop second particle into the shooter

        shootBall(); // shoot second particle

        shootBall();

        // Retract both servos now both particles have been shot into the center vortex
        redServo.setPosition(RED_SERVO_RETRACTED);
        blueServo.setPosition(BLUE_SERVO_RETRACTED);
        sleep(500);

        closeBallShooter(330);

        if(colorString.equalsIgnoreCase("red")) {
            spinTurn((0), (float) 0.25, AutonomousVelVort.moveDirection.RIGHT); //turns left towards beacon based on absolute orientation reading
        } else{
            spinTurn((0), (float) 0.25, AutonomousVelVort.moveDirection.LEFT); //turns left towards beacon based on absolute orientation reading

        }

        sleep(1000);


        moveStraight(0.8, 90, true);

        sleep(750);

        if(colorString.equalsIgnoreCase("red")) {
            spinTurn((-50), (float) 0.25, AutonomousVelVort.moveDirection.LEFT); //turns left towards beacon based on absolute orientation reading
        } else{
            spinTurn((50), (float) 0.25, AutonomousVelVort.moveDirection.RIGHT); //turns left towards beacon based on absolute orientation reading

        }

        moveStraight(0.8, 90, true);

        sleep(1000);

        alignRobot(0.3);



    }

}
