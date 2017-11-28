package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="EncoderMovementWITM", group="Autonomous")
//@Disabled
public class EncoderPlagiarismWITM extends LinearOpMode {
    // OpticalDistanceSensor opticalDistanceSenso
    //Drive motors
    DcMotor rightFront,leftFront, rightBack, leftBack;
    int leftFrontPos, leftBackPos, rightFrontPos, rightBackPos;

    @Override
    public void runOpMode() throws InterruptedException {

        // opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");

        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftBack = hardwareMap.dcMotor.get("leftBack");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();
        // Reset enoders to zero
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //waitOneFullHardwareCycle(); // We use these in attempt to gain stability.


        leftFront.setTargetPosition(2440);
        rightFront.setTargetPosition(2440);
        leftBack.setTargetPosition(2440);
        rightBack.setTargetPosition(2440);

        leftFrontPos = leftFront.getCurrentPosition();
        leftBackPos = leftBack.getCurrentPosition();
        rightFrontPos = rightFront.getCurrentPosition();
        rightBackPos = rightBack.getCurrentPosition();

        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(leftBackPos < 2440 && leftFrontPos < 2440 && rightBackPos < 2440 && rightFrontPos < 2440){
            leftFront.setPower(0.5);
            rightFront.setPower(0.5);
            leftBack.setPower(0.5);
            rightBack.setPower(0.5);

            leftFrontPos = leftFront.getCurrentPosition();
            leftBackPos = leftBack.getCurrentPosition();
            rightFrontPos = rightFront.getCurrentPosition();
            rightBackPos = rightBack.getCurrentPosition();

            telemetry.addData("2 ", "motorFrontLeft:  " + String.format("%d", leftFront.getTargetPosition()));
            telemetry.addData("3 ", "motorFrontRight:  " + String.format("%d", rightFront.getTargetPosition()));
            telemetry.addData("4 ", "leftBack:  " + String.format("%d", leftBack.getTargetPosition()));
            telemetry.addData("5 ", "rightBack:  " + String.format("%d", rightBack.getTargetPosition()));

        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

    }
}