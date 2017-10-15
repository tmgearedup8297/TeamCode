package org.firstinspires.ftc.teamcode;
import java.util.*;
import java.text.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name = "BLUEDelawareState", group = "BLUEDelawareState")
@Disabled

public class BLUEdelawareState extends DelawareState {

    @Override
    public void runOpMode() throws InterruptedException {

        initializeHardware();

        setAttributes(1120, 4, (float) 12);

        waitForStart();

        activateDelawareState("blue");

        exitOpMode();

    }

}

