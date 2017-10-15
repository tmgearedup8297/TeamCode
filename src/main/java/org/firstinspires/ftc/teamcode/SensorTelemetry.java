package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "SensorTelemetry", group = "SensorTelemetry")

public class SensorTelemetry extends TeleopVelVort {


    @Override
    public void runOpMode() throws InterruptedException{

        initializeHardware();

        setAttributes(1120, 4, (float) 12);

        waitForStart();

        while(opModeIsActive()) {

            setMotorPower();

            powerShooter();

            setIntakePower();

            giveSensorFeedback();
        }

    }


}
