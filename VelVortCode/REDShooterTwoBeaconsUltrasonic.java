package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "REDShooter-TWO-Beacons-Ultrasonic", group = "1")

public class REDShooterTwoBeaconsUltrasonic extends ShooterTWOBeaconsUltrasonic {

    @Override
    public void runOpMode() throws InterruptedException {

        initializeHardware();

        setAttributes(2240, 4, (float) 12);

        waitForStart();

        activateShooterTWOBeaconsUltrasonic("red");

        exitOpMode();

    }

}

