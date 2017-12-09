package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static android.os.SystemClock.sleep;

/**
 * Created by 770742 on 11/26/2017.
 */
@Autonomous(name = "MovebotbyTime", group = "Autonomous")
public class MovebotbyTime  extends LinearOpMode {
    static final double power=0.25;
    static final int time=1500;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();
        leftBack.setPower(power);
        rightFront.setPower(power);
        leftFront.setPower(power);
        rightFront.setPower(power);
        sleep(time);
        leftBack.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(-power);
        rightFront.setPower(-power);
        leftFront.setPower(-power);
        rightFront.setPower(-power);
        sleep(550);
        leftBack.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);

    }
}
//You'll have to adjust the power variable to account for all 4 wheels
//Extends the Master Device class from last year