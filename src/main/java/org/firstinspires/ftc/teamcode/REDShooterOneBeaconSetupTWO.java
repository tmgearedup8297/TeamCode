package org.firstinspires.ftc.teamcode;
import java.util.*;
import java.text.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "REDShooter-ONE-Beacon-Setup-2", group = "5")

public class REDShooterOneBeaconSetupTWO extends ShooterOneBeaconSetupTWO {

    @Override
    public void runOpMode() throws InterruptedException {

        initializeHardware();

        setAttributes(1120, 4, (float) 12);

        waitForStart();

        activateShooterOneBeaconSetupTWO("red");

        exitOpMode();

    }

}

