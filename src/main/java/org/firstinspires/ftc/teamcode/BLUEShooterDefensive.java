package org.firstinspires.ftc.teamcode;
import java.util.*;
import java.text.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "BLUEShooterDEFENSIVE", group = "7")

public class BLUEShooterDefensive extends ShooterDefensive {

    @Override
    public void runOpMode() throws InterruptedException {

        initializeHardware();

        setAttributes(1120, 4, (float) 12);

        waitForStart();

        activateShooterDefensive("blue");

        exitOpMode();

    }

}
