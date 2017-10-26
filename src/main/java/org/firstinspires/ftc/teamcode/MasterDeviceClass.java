package org.firstinspires.ftc.teamcode;
import java.util.Date;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;


import com.qualcomm.hardware.adafruit.*;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

// Nav-X packages
import com.kauailabs.navx.ftc.navXPIDController;
import com.kauailabs.navx.ftc.AHRS;


public abstract class MasterDeviceClass extends LinearOpMode{

    static final double INITIAL_VOLTAGE = 13.8;






    BNO055IMU dof;
    float angle_0;
    int realOrientation; // used by DOF sensor

    //might not need
    ModernRoboticsI2cRangeSensor BLUEsensorRange;
    ModernRoboticsI2cRangeSensor REDsensorRange;
    TouchSensor touchSensor;


    float wheelDia; //wheel diameter
    int ticksPerRotation; //ticks per rotation
    float rotationsPerInch; //rotations per inch
    float wheelBase; //wheel base
    float turnsPerRotation; // turns per rotation

    public enum moveDirection {LEFT, RIGHT, FORWARD, BACK, UP, DOWN} //set an enum for move direction (used in spinTurn, and moveStraight

    public enum gyro {ADAFRUIT, NAVX} //set an enum for move direction (used in spinTurn, and moveStraight

    public gyro choose_gyro = gyro.NAVX;




    //Misc Constants
    static final float PI = (float) 3.1415; //constant for Pi
    static final int DEGREES_PER_FULL_TURN = 360; // degrees covered in a full turn of the robot
    static final double sprocketReductionRatio = 0.5; // Sprocket reduction ration

     static final int ticksRotation = 2240;

    // Angle adjustment  THRESHOLDS
    static final int GYRO_RED_THRESHOLD = -2; // RED Angle is always -ve

    static final int GYRO_BLUE_THRESHOLD = 2; // BLUE Angle is always +ve


    static final double STRAFE_LONG = 8.5;
    static final double STRAFE_SHORT = 4.0;

    static final double STRAFE_SECOND = 5;


    DcMotor motorRightBack;
    DcMotor motorLeftBack;
    DcMotor motorRightFront;
    DcMotor motorLeftFront;
    Servo jewelTool, left, right;

    int initialGyroHeading;






    // NAV-X GYRO STUFF

    final int NAVX_DIM_I2C_PORT = 4; //PORT # on the device controller where NAVX is plugged - IMPORTANT! since it does not use the FTC config file
    public AHRS navx_device;
    public navXPIDController yawPIDController;

    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    final int NAVX_DEVICE_TIMEOUT_MS = 5000;

    public boolean calibration_complete = false;



    public boolean returnInitState() throws InterruptedException {
        if(isStopRequested()){

            motorRightBack.setPower(0);
            motorRightFront.setPower(0);
            motorLeftBack.setPower(0);
            motorRightFront.setPower(0);
            return true;


        } else{
            return false;
        }
    }

    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }




        return result;
    }

    public double getPowerMultiplier(double power){

        //Based on the current voltage, multiply the power

        double currentVoltage = this.getBatteryVoltage();

        double powerMultiplier = (((((1 - ((currentVoltage)/(INITIAL_VOLTAGE)))) *1.3) + 1)); //Find the percentage of the starting voltage, which should be measured at the beginning of the match

        //double powerMultiplier = ((power) * (java.lang.Math.pow(((1 - ((currentVoltage)/(INITIAL_VOLTAGE)))) + 1)), 3);

        if(powerMultiplier<= 0) {
            return power;
        }
        telemetry.addData("power multiplier", powerMultiplier);
        telemetry.update();
        return powerMultiplier;
    }

    public void initializeHardware() throws InterruptedException {

        telemetry.addData("HARDWARE INIT STARTED", 0);

        telemetry.update();
        returnInitState();


        // Drive Motors
        motorRightBack = hardwareMap.dcMotor.get("rightback");


        motorLeftFront = hardwareMap.dcMotor.get("leftfront");

        motorLeftBack = hardwareMap.dcMotor.get("leftback");

        motorRightFront = hardwareMap.dcMotor.get("rightfront");

        // Reverse direction of one front and one back motor
        motorRightBack.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.REVERSE);

        // Servo Motors
        jewelTool = hardwareMap.servo.get("jewelTool");
        left = hardwareMap.servo.get("leftServo");
        right = hardwareMap.servo.get("rightServo");
        right.setDirection(Servo.Direction.REVERSE);
        left.setPosition(0);
        right.setPosition(0);


        // Set all the motor power to ZERO
        motorRightBack.setPower(0);
        motorRightFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);


        // Initialize all the RANGE Sensors

        BLUEsensorRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "bluerange");
        REDsensorRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "redrange");



        touchSensor = hardwareMap.touchSensor.get("touch");


        telemetry.addData("Starting ADAFRUIT gyro INIT", 0);
        telemetry.update();

        returnInitState();

        // ADAFruit 9DOF Initialization and Calibration
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "9DOF";
        returnInitState();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "9dof".
        dof = hardwareMap.get(BNO055IMU.class, "dof");
        dof.initialize(parameters);

        returnInitState();
        sleep(10000);
        telemetry.addData("ADAFRUIT", " - Init COMPLETED");
        telemetry.update();

        // Init the DOF sensor
        Orientation angle_orientation = dof.getAngularOrientation();
        returnInitState();

        angle_0 = angle_orientation.firstAngle; // get the Z heading and store for future reference

        //NAVX MICRO INIT
        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("Device Interface Module 1"), NAVX_DIM_I2C_PORT, AHRS.DeviceDataType.kProcessedData, NAVX_DEVICE_UPDATE_RATE_HZ);

        //  navX-Micro Calibration completes automatically ~15 seconds after it is powered on, as long as the device is still.  To handle the case where the
        // navX-Micro has not been able to calibrate successfully, hold off using the navX-Micro Yaw value until calibration is complete.
        while (!calibration_complete) {

            calibration_complete = navx_device.isCalibrating();
            telemetry.addData("navX-Micro", " - Calibration STARTED");
            telemetry.update();


            if (!calibration_complete) {
                telemetry.addData("navX-Micro", "Startup Calibration in Progress");
            }
        }
        navx_device.zeroYaw(); // set the yaw value to zero during init so everything is relative to this starting position

        telemetry.addData("NAVX-Micro", " - Calibration COMPLETED");
        telemetry.update();



        telemetry.addData("SERVO INIT", "COMPLETE");
        telemetry.update();

        returnInitState(); // makes sure you can exit in the middle of Init() if the stop is pressed


        // Wait for the touch sensor to be pressed to see if there is an override for the gyro
        Date start_date = new Date();
        long start_time = start_date.getTime();


        Date current_date = new Date ();

        while ((current_date.getTime() - start_time) < NAVX_DEVICE_TIMEOUT_MS){

            if (touchSensor.isPressed()) {
                choose_gyro = gyro.ADAFRUIT;

                telemetry.addData("ADAFRUIT Gyro", " SELECTED via Touch Sensor");
                telemetry.update();
                break;
            }

            current_date = new Date ();
            telemetry.addData("If NAVX OFF - PRESS TOUCH SENSOR NOW!! ", current_date.getTime());
            telemetry.update();
        }

        telemetry.addData("======== HARDWARE INIT COMPLETE; Gyro Selected:", choose_gyro);
        telemetry.update();

    }

    public void setAttributes(int pTicksPerRotation, float pWheelDia, float pWheelBase) {

        //assign values
        wheelDia = pWheelDia;
        wheelBase = pWheelBase;
        ticksPerRotation = pTicksPerRotation;
        rotationsPerInch = 1 / (PI * wheelDia);
        turnsPerRotation = wheelBase / wheelDia; //Calculate how many ticks the robot moves in one rotation (360 degree turn) by dividing wheel base by the wheel diameter.

    }



}
