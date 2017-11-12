package org.firstinspires.ftc.teamcode;

/**
 * Created by govindb on 1/7/17.
 */
public abstract class DelawareState extends AutonomousVelVort {


    public void activateDelawareState(String colorString) throws InterruptedException{

        sleep(15000);

        moveStraight(0.4, 6, true);

        sleep(300);

        if(colorString.equalsIgnoreCase("red")) {
            spinTurnUsingDOF((-12), (float) 0.25, AutonomousVelVort.moveDirection.LEFT); //turns left towards beacon based on absolute orientation reading
        } else{
            spinTurnUsingDOF((12), (float) 0.25, AutonomousVelVort.moveDirection.RIGHT); //turns left towards beacon based on absolute orientation reading

        }

        sleep(500);

        shootBall();


        if(colorString.equalsIgnoreCase("red")){
            spinTurnUsingDOF((-56), (float) 0.25, AutonomousVelVort.moveDirection.LEFT); //turns left towards beacon based on absolute orientation reading

        } else{
            spinTurnUsingDOF((56), (float) 0.25, AutonomousVelVort.moveDirection.RIGHT); //turns left towards beacon based on absolute orientation reading

        }

        sleep(200);

        moveStraight(0.7, 90, true);

        sleep(1500);


        if(colorString.equalsIgnoreCase("red")) {
            spinTurnUsingDOF((-12), (float) 0.15, AutonomousVelVort.moveDirection.RIGHT); //turns right in order to align with white line
        } else{
            spinTurnUsingDOF((12), (float) 0.15, AutonomousVelVort.moveDirection.LEFT); //turns right in order to align with white line

        }
        sleep(100);


        moveStraight(0.5, 10, false); // partially parks on the central vortex platform

    }

}
