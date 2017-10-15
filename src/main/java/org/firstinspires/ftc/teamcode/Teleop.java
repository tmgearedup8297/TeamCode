package org.firstinspires.ftc.teamcode;
//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;

import com.qualcomm.hardware.bosch.BNO055IMU;

//import com.qualcomm.hardware.adafruit.BNO055IMU;

//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "TELEOP", group = "TELEOP")

public class Teleop extends OpMode {

    //ColorSensor bottomcolor;

    //ColorSensor BLUEfrontColor;
    //ColorSensor BLUEbackColor;
    //ColorSensor REDfrontColor;
    //ColorSensor REDbackColor;

    Servo blueServo;
    //Servo redServo;
    //Servo capBallServo;


    //GyroSensor gyrosensor;
    //BNO055IMU dof;
    //float angle_0;

    //ModernRoboticsI2cRangeSensor BLUEsensorRange;
    //ModernRoboticsI2cRangeSensor REDsensorRange;

    DcMotor intake;

    //int var = 0;
    //int previousVar = 0;


    //float wheelDia; //wheel diameter
    //int ticksPerRotation; //ticks per rotation
    //float rotationsPerInch; //rotations per inch
    //float wheelBase; //wheel base
    //float turnsPerRotation; // turns per rotation

    public enum moveDirection {LEFT, RIGHT, FORWARD, BACK, UP, DOWN} //set an enum for move direction (used in spinTurn, and moveStraight

    /*static final int
            whiteARGBMin = 25000000,
            whiteARGBMax = 60000000,
            blueARGB = 15000000,
            redARGB = 29000000,
            whiteARGB = 150000000;

    //Misc Constants
    static final float PI = (float) 3.1415; //constant for Pi
    static final int DEGREES_PER_FULL_ = 360; // degrees covered in a full turn of the robot
    static final double sprocketReductionRatio = 0.5; // Sprocket reduction ration

    static final double Kp = 0.005; // This is the proportional gain,  defines how much to respond to an error

    static final double Ki = 0; // This is the integrator gain,  defines how much the accumulated error will contribute to the motor correction
*/

    DcMotor motorRightBack;
    DcMotor motorLeftBack;
    DcMotor motorRightFront;
    DcMotor motorLeftFront;
    DcMotor shooter;
    DcMotor blueLift;
    DcMotor redLift;

    float x1, x2, y1, y2;
    float x1_zero, x2_zero, y1_zero, y2_zero;

    static final double RED_SERVO_EXTENDED = 0.78;
    static final double RED_SERVO_RETRACTED = 0.45;

    static final double BLUE_SERVO_EXTENDED = 0.85;
    static final double BLUE_SERVO_RETRACTED = 0.52;

    //static final double CAP_BALL_SERVO_CLOSED = 0.1;

    @Override
    public void init() {


        telemetry.addData("TELEOP Hardware Init Started.......", 0);

        telemetry.update();

        // Drive Motors
        motorRightBack = hardwareMap.dcMotor.get("rightback");

        motorLeftFront = hardwareMap.dcMotor.get("leftfront");

        motorLeftBack = hardwareMap.dcMotor.get("leftback");

        motorRightFront = hardwareMap.dcMotor.get("rightfront");

        // Reverse direction of one front and one back motor
        motorRightBack.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.REVERSE);

        Servo Motors;
        blueServo = hardwareMap.servo.get("blueservo");

        blueServo.setDirection(Servo.Direction.REVERSE); // Reverse the blue servo

        //redServo = hardwareMap.servo.get("redservo");

       // capBallServo = hardwareMap.servo.get("capballservo");


        // Shooter Motor
        shooter = hardwareMap.dcMotor.get("shooter");

        // Intake Motor
        intake = hardwareMap.dcMotor.get("intake");

        //Lift motors
        blueLift = hardwareMap.dcMotor.get("blueLift");
        redLift = hardwareMap.dcMotor.get("redLift");

        redLift.setDirection(DcMotor.Direction.REVERSE); // Reverse the red lift motor



        x1_zero = gamepad1.left_stick_x;
        x2_zero = gamepad1.right_stick_x;
        y1_zero = gamepad1.left_stick_y;
        y2_zero = gamepad1.right_stick_y;


        // set up the init positions for all three servos
       blueServo.setPosition(BLUE_SERVO_RETRACTED);
        //redServo.setPosition(RED_SERVO_RETRACTED);
        //capBallServo.setPosition(CAP_BALL_SERVO_CLOSED);

        motorRightBack.setPower(0);
        motorRightFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        shooter.setPower(0);
        intake.setPower(0);

        telemetry.addData(" TELEOP HARDWARE INIT COMPLETE", "");


    }

    @Override
    public void loop() {


        // retrieve and store all the stick values first
        x1 = gamepad1.left_stick_x;
        x2 = gamepad1.right_stick_x;
        y1 = gamepad1.left_stick_y;
        y2 = gamepad1.right_stick_y;

        // Drive Motor Controls - mapped to gamepad 1 left and right sticks
        motorLeftFront.setPower((-y1 + x2 + x1));

        motorRightFront.setPower((-y1 - x2 - x1));

        motorRightBack.setPower((-y1 - x2 + x1));

        motorLeftBack.setPower((-y1 + x2 - x1));

        if(gamepad1.x ==true){
            blueServo.setPosition(BLUE_SERVO_EXTENDED);
        }
        else if(gamepad1.b == true){
            blueServo.setPosition(BLUE_SERVO_RETRACTED);
        }

        //Ball Intake Control - mapped to gamepad 1 triggers
        if (gamepad1.right_trigger > gamepad1.left_trigger) {
            intake.setPower(Math.abs(gamepad1.right_trigger));
        } else if (gamepad1.left_trigger > gamepad1.right_trigger) {
            intake.setPower(-(gamepad1.left_trigger));
        } else {
            intake.setPower(0);
        }

        //Shooter control mapped to gamepad 2 bumpers
        if (gamepad1.right_bumper==true) {
            shooter.setPower(1);
        }else{
            shooter.setPower(0);
        }
    }
    //@Override
    public void stop() {
        motorRightBack.setPower(0);
        motorRightBack.close();
        motorRightFront.setPower(0);
        motorRightFront.close();
        motorLeftBack.setPower(0);
        motorLeftBack.close();
        motorRightFront.setPower(0);
        motorRightFront.close();

        shooter.close();

        //blueServo.close();
        //redServo.close();
        //capBallServo.close();

        intake.close();
        redLift.close();
        blueLift.close();

    }

}