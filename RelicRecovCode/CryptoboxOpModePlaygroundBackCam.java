
package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.CryptoboxDetector;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="DogeCV Cryptobox Detector Playground Back", group="DogeCV")

public class CryptoboxOpModePlaygroundBackCam extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


     private CryptoboxDetector cryptoboxDetector = null;
    private ModernRoboticsI2cRangeSensor rangeSensor;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");


        cryptoboxDetector = new CryptoboxDetector();
        cryptoboxDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0);
        cryptoboxDetector.rotateMat = false;
        cryptoboxDetector.setDetectionMode("blue");

        cryptoboxDetector.enable();
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");


    }

    @Override
    public void init_loop() {
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        runtime.reset();


    }

    @Override
    public void loop() {





        //telemetry.addData("Column Left ",  cryptoboxDetector.getCryptoBoxLeftPosition());
        telemetry.addData("Column Center ",  cryptoboxDetector.getCryptoBoxCenterPosition());
        //telemetry.addData("Column Right ",  cryptoboxDetector.getCryptoBoxRightPosition());

        int leftToCenter = cryptoboxDetector.getCryptoBoxLeftPosition()-cryptoboxDetector.getCryptoBoxCenterPosition();
        int centerToRight = cryptoboxDetector.getCryptoBoxCenterPosition()-cryptoboxDetector.getCryptoBoxRightPosition();
        telemetry.addData("Left To Center " , leftToCenter);
        telemetry.addData("Center To Right", centerToRight);

        telemetry.addData("Image width" , cryptoboxDetector.getWidth());
        telemetry.addData("Image height" , cryptoboxDetector.getHeight());

        telemetry.addData("Left to Center Dist (in)" , 0.03295896328293736501079913606911*leftToCenter);
        telemetry.addData("Center to Right Dist(in)" , 0.03295896328293736501079913606911*centerToRight);

        telemetry.addData("Center dist new method", cryptoboxDetector.getDistancePos1(1));
        telemetry.addData("Right dist new method", cryptoboxDetector.getDistancePos1(2));
        telemetry.addData("Left dist new method", cryptoboxDetector.getDistancePos1(0));


        telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
        telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.update();


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        cryptoboxDetector.disable();
    }

}
