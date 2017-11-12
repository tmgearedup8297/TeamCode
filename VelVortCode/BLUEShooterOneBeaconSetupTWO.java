package org.firstinspires.ftc.teamcode;
import java.util.*;
import java.text.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "BLUEShooter-ONE-Beacon-Setup-2", group = "5")

public class BLUEShooterOneBeaconSetupTWO extends ShooterOneBeaconSetupTWO {

    @Override
    public void runOpMode() throws InterruptedException {

        initializeHardware();

        setAttributes(1120, 4, (float) 12);

        waitForStart();

        activateShooterOneBeaconSetupTWO("blue");

        exitOpMode();

    }

}

