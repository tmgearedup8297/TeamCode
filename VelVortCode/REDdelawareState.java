package org.firstinspires.ftc.teamcode;
import java.util.*;
import java.text.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;



@Autonomous(name = "REDdelawareState", group = "REDdelawareState")
@Disabled
public class REDdelawareState extends DelawareState {

    @Override
    public void runOpMode() throws InterruptedException {

        initializeHardware();

        setAttributes(1120, 4, (float) 12);

        waitForStart();

        activateDelawareState("red");

        exitOpMode();

    }

}

