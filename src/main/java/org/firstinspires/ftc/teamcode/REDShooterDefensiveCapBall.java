package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "RED DEFENSIVE CAP BALL", group = "4")

public class REDShooterDefensiveCapBall extends ShooterDefensiveCapBall {

    @Override
    public void runOpMode() throws InterruptedException {

        initializeHardware();

        setAttributes(2240, 4, (float) 12);

        waitForStart();

        activateShooterDefensiveCapBall("red");

        exitOpMode();

    }

}

