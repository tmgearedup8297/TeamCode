package org.firstinspires.ftc.teamcode;
import java.util.*;
import java.text.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "BLUEShooter-ONE-Beacon", group = "2")

public class BLUEShooterOneBeacon extends ShooterOneBeacon {

    @Override
    public void runOpMode() throws InterruptedException {

        initializeHardware();

        setAttributes(1120, 4, (float) 12);

        waitForStart();

        activateShooterOneBeacon("blue");

        exitOpMode();

    }

}

