package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "BLUEShooter-TWO-Beacons", group = "6")

public class BLUEShooterTWOBeacons extends ShooterTWOBeacons {

    @Override
    public void runOpMode() throws InterruptedException {

        initializeHardware();

        setAttributes(2240, 4, (float) 12);

        waitForStart();

       activateShooterBeacon("blue");

        exitOpMode();

    }

}

