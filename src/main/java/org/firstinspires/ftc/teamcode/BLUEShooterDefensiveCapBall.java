package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "BLUE DEFENSIVE CAP BALL", group = "4")

public class BLUEShooterDefensiveCapBall extends ShooterDefensiveCapBall {

    @Override
    public void runOpMode() throws InterruptedException {

        initializeHardware();

        setAttributes(2240, 4, (float) 12);

        waitForStart();

        activateShooterDefensiveCapBall("blue");

        exitOpMode();

    }

}
