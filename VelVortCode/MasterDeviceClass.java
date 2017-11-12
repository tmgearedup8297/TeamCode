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

    ColorSensor bottomcolor;

    ColorSensor BLUEfrontColor;
    ColorSensor BLUEbackColor;
    ColorSensor REDfrontColor;
    ColorSensor REDbackColor;


    Servo blueServo;
    Servo redServo;
    Servo capBallServo;


    BNO055IMU dof;
    float angle_0;
    int realOrientation; // used by DOF sensor


    ModernRoboticsI2cRangeSensor BLUEsensorRange;
    ModernRoboticsI2cRangeSensor REDsensorRange;

    DcMotor intake;

    TouchSensor touchSensor;


    float wheelDia; //wheel diameter
    int ticksPerRotation; //ticks per rotation
    float rotationsPerInch; //rotations per inch
    float wheelBase; //wheel base
    float turnsPerRotation; // turns per rotation

    public enum moveDirection {LEFT, RIGHT, FORWARD, BACK, UP, DOWN} //set an enum for move direction (used in spinTurn, and moveStraight

    public enum gyro {ADAFRUIT, NAVX} //set an enum for move direction (used in spinTurn, and moveStraight

    public gyro choose_gyro = gyro.NAVX;


    static final int    whiteARGBMin = 1000000,
            whiteARGBMax = 60000000,
            blueARGB = 15000000,
            redARGB = 29000000,
            whiteARGB = 150000000;

    //Misc Constants
    static final float PI = (float) 3.1415; //constant for Pi
    static final int DEGREES_PER_FULL_TURN = 360; // degrees covered in a full turn of the robot
    static final double sprocketReductionRatio = 0.5; // Sprocket reduction ration

    static final double Kp = 0.005; // This is the proportional gain,  defines how much to respond to an error

    static final double Ki = 0; // This is the integrator gain,  defines how much the accumulated error will contribute to the motor correction

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
    DcMotor shooter;

    int initialGyroHeading;

    // I2C Addresses of all the Color Sensors used on the robot
    I2cAddr i2CAddressBottomColor = I2cAddr.create8bit(0x3a);
    I2cAddr i2CAddressBLUEFrontColor = I2cAddr.create8bit(0x3c);
    I2cAddr i2CAddressBLUEBackColor = I2cAddr.create8bit(0x3e);
    I2cAddr i2CAddressREDBackColor = I2cAddr.create8bit(0x3e);
    I2cAddr i2CAddressREDFrontColor = I2cAddr.create8bit(0x3c);


    // SERVO Positions
    static final double RED_SERVO_EXTENDED = 0.78;
    static final double RED_SERVO_RETRACTED = 0.45;

    static final double BLUE_SERVO_EXTENDED = 0.90;
    static final double BLUE_SERVO_RETRACTED = 0.52;

    static final double CAP_BALL_SERVO_LOCKED = 0.1;

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
            shooter.setPower(0);
            intake.setPower(0);


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
        blueServo = hardwareMap.servo.get("blueservo");
        blueServo.setDirection(Servo.Direction.REVERSE); // reverse the blue servo

        redServo = hardwareMap.servo.get("redservo");

        capBallServo = hardwareMap.servo.get("capballservo");

        // Shooter Motor
        shooter = hardwareMap.dcMotor.get("shooter");

        intake = hardwareMap.dcMotor.get("intake");

        // Set all the motor power to ZERO
        motorRightBack.setPower(0);
        motorRightFront.setPower(0);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        shooter.setPower(0);
        intake.setPower(0);

        // Initialize all the RANGE Sensors
        bottomcolor = hardwareMap.colorSensor.get("bottomcolor");
        BLUEsensorRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "bluerange");
        REDsensorRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "redrange");

        // Initialize all the COLOR Sensors
        BLUEfrontColor = hardwareMap.colorSensor.get("bluefrontColor");
        BLUEbackColor = hardwareMap.colorSensor.get("bluebackColor");
        REDbackColor = hardwareMap.colorSensor.get("redbackColor");
        REDfrontColor = hardwareMap.colorSensor.get("redfrontColor");


        // Set up the I2C addresses for all the MR color Sensors
        BLUEfrontColor.setI2cAddress(i2CAddressBLUEFrontColor);
        BLUEbackColor.setI2cAddress(i2CAddressBLUEBackColor);
        REDbackColor.setI2cAddress(i2CAddressREDBackColor);
        REDfrontColor.setI2cAddress(i2CAddressREDFrontColor);
        bottomcolor.setI2cAddress(i2CAddressBottomColor);


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


        // Set the servos to retracted position
        blueServo.setPosition(BLUE_SERVO_RETRACTED);
        redServo.setPosition(RED_SERVO_RETRACTED);

        // Lock the cap ball lifter
        capBallServo.setPosition(CAP_BALL_SERVO_LOCKED);

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
