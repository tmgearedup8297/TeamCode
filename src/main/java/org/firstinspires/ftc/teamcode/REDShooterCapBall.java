package org.firstinspires.ftc.teamcode;
import java.util.*;
import java.text.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "REDShooterCapBall", group = "3")

public class REDShooterCapBall extends ShooterCapBall {

    @Override
    public void runOpMode() throws InterruptedException {

        initializeHardware();

        setAttributes(1120, 4, (float) 12);

        waitForStart();

        activateShooterCapBall("red");

        exitOpMode();

    }

}