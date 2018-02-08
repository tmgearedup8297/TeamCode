package org.firstinspires.ftc.teamcode.Autonomous;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.CryptoboxDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by BenLoan on 12/12/2017.
 * Where you can upload your test code to be tested
 */

//@Autonomous(name = "Autonomous Test")
public class AutonomousTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        CryptoboxDetector detector = new CryptoboxDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.downScaleFactor = 0.4;
        detector.setColor("Blue");

        waitForStart();

        detector.enable();

        runtime.reset();

        List<Double> distances = new ArrayList<>();

        while (opModeIsActive() && runtime.seconds() < 5) {
            if (detector.getDistanceToMove() > -7.5 && detector.getDistanceToMove() < 7.5) {
                distances.add(detector.getDistanceToMove());
            }
            telemetry.addData("Distance to Move: ", detector.getDistanceToMove());
            telemetry.update();
        }

        double sum = 0;

        for (double distance: distances) {
            sum += distance;
        }

        double distanceToMove = sum / distances.size();

        if (distanceToMove < 7.5 && distanceToMove > -7.5) {
            if (distanceToMove < 0) {
                robot.forwardByDistance(-distanceToMove);
            } else if (distanceToMove > 0) {
                robot.backwardByDistance(distanceToMove);
            }
            robot.ledStrip(Robot.LED.Green);

            telemetry.addData("Distance Moved: ", (int) distanceToMove);
        } else {
            if (distanceToMove == 100) {
                telemetry.addData("Distance Moved: ", "No Cryptobox");
                robot.ledStrip(Robot.LED.Red);
            } else {
                telemetry.addData("Distance Moved: ", "Double Stacked Box");
                robot.ledStrip(Robot.LED.Red);
            }
        }

        telemetry.update();

        detector.disable();

        sleep(5000);
    }
}

