
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Teleop.moveDirection;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.io.InterruptedIOException;
import java.util.Locale;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto Drive methods test", group="Linear Opmode")
//@Disabled
public class AutoDriveMethodsTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    private final int TICKS_PER_INCH=36;


    private int[] zeroPos = {0, 0, 0, 0};
    private double[] targetDist = {24, 24, 24, 24};
    private int[] targetClicks = {0, 0, 0, 0};

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    private float initAngle;



    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override public void runOpMode() {

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        /**/
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //shoulder.setPosition(spos);
        // elbow.setPosition(epos);
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        //composeTelemetry();


        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        initAngle = angles.firstAngle;
        float deltaAngle = 0;
        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Loop and update the dashboard

        // Wait for the game to start (driver presses PLAY)

        runtime.reset();

        //strafeDistLeft(50, 50, 50, 50);
        strafeDistRight(50, 50, 50, 50);   //Strafes right


        //}


        // run until the end of the match (driver presses STOP)

    }
    public int formatDOFValue(int headingDof) {
        return -headingDof;
    }
    public void spinTurnUsingDOF(float degrees, moveDirection direction) throws InterruptedException {

        Orientation current_orientation = imu.getAngularOrientation();
        int realOrientation = (int) (current_orientation.firstAngle - initAngle);

        // LEFT turn values always have to come in with -ve DEGREES
        if (direction == moveDirection.RIGHT) {
            while ((formatDOFValue(realOrientation) < degrees) && opModeIsActive()) {

                leftBack.setPower(.1);
                leftFront.setPower(.14);
                rightFront.setPower(-.14);
                rightBack.setPower(-.1);

                // refresh the  value inside the while loop

                current_orientation = imu.getAngularOrientation();
                realOrientation = (int) (current_orientation.firstAngle - initAngle);

            }
            brake();
            telemetry.addData("DOF gyro value WHILE RIGHT", realOrientation);
            telemetry.update();

            // RIGHT turn values always have to come in with +ve DEGREES


        } else if (direction == moveDirection.LEFT) {

            while ((formatDOFValue(realOrientation) > degrees) && opModeIsActive()) {
                leftBack.setPower(-.1);
                leftFront.setPower(-.14);
                rightFront.setPower(.14);
                rightBack.setPower(.1);

                // refresh the  value inside the while loop

                current_orientation = imu.getAngularOrientation();
                realOrientation = (int) (current_orientation.firstAngle - initAngle);

            }
            brake();

            telemetry.addData("DOF gyro value WHILE LEFT", realOrientation);
            telemetry.update();
        }

        //Stop the motors once both if statements are completed.
        brake();

    }

    public void correction(float initAngle, int yCorr, int xCorr){
            float deltaAngle=angles.firstAngle-initAngle;  //Fixes the angle discrepancy
            try{
                if(deltaAngle<0){
                    spinTurnUsingDOF(deltaAngle, moveDirection.LEFT);
                }
                else {
                    spinTurnUsingDOF(deltaAngle, moveDirection.RIGHT);
                }
            }catch(InterruptedException e){
                e.printStackTrace();
            }

            int yDist =(int)(yCorr*Math.sin(deltaAngle));
            moveDistBack(yDist, yDist, yDist, yDist);



        telemetry.addData("Distance is off", "");
        telemetry.update();

        }


    public void strafeDistRight(int leftFrontTargetDist, int leftBackTargetDist, int rightFrontTargetDist, int rightBackTargetDist) {

        targetDist[0] = leftFrontTargetDist; //getting target distances
        targetDist[1] = leftBackTargetDist;
        targetDist[2] = rightFrontTargetDist;
        targetDist[3] = rightBackTargetDist;

        zeroPos[0] = leftFront.getCurrentPosition();//getting current positions of the motors
        zeroPos[1] = leftBack.getCurrentPosition();//not gonna reset the motors, treating current
        zeroPos[2] = rightFront.getCurrentPosition();//positions as 0
        zeroPos[3] = rightBack.getCurrentPosition();

        targetClicks[0] = (int) (targetDist[0] * TICKS_PER_INCH);//converting the inches they have to move into encoder ticks
        targetClicks[1] = (int) (targetDist[1] * TICKS_PER_INCH);
        targetClicks[2] = (int) (targetDist[2] * TICKS_PER_INCH);
        targetClicks[3] = (int) (targetDist[3] * TICKS_PER_INCH);


        leftFront.setPower(-.1);//setting power to wheels; robot starts moving
        leftBack.setPower(.14);
        rightFront.setPower(-.14);
        rightBack.setPower(.1);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//setting mode to run using encoder
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//might be useless code, saw it used in someone else's so its here
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//gonna check it out later
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        float initAngle=angles.firstAngle;//find the heading of the robot before moving, trying to preserve this value throughout the motion
        float deltaAngle=0;//the angle that we deviate
        int correctionCompensation=0;//This variable makes up for the motors motion during correction. Because leftFront will be used in
                                        // correction, its encoder vals will be messed up, making the while loop end early, so we
                                        //add enc vals to its current position so it'll end at the right time
        while (opModeIsActive() && leftFront.getCurrentPosition()+correctionCompensation>zeroPos[0]-targetClicks[0]) {
            deltaAngle=angles.firstAngle-initAngle;
            telemetry.addData("DeltaAngle:", " "+deltaAngle);
            telemetry.addData("CurrentAngle:", angles.firstAngle);
           //telemetry.addData("");
            if(deltaAngle>19){ //arbitrary
                try {
                    spinTurnUsingDOF(-deltaAngle, moveDirection.RIGHT);
                    correctionCompensation+=12*Math.PI *(2*deltaAngle/360);
                    telemetry.addData("Correction Compensation", ""+correctionCompensation);
                }catch(Exception e){
                    e.printStackTrace();
                }
            }
        }
        brake();
        /*int oldEnc =100;
        int totalXChange=0;
        while (opModeIsActive() && totalXChange<=leftFrontTargetDist) {

            int currentEnc = leftFront.getCurrentPosition();

            if(leftFront.getCurrentPosition()%1000<=50||leftFront.getCurrentPosition()%1000>=50) {
                correction(0, currentEnc-oldEnc, currentEnc-oldEnc);
                totalXChange+=(int)((currentEnc-oldEnc)*Math.cos(angles.firstAngle));
                oldEnc=currentEnc;
            }

        }*/


    }




    public void strafeDistLeft(int leftFrontTargetDist, int leftBackTargetDist, int rightFrontTargetDist, int rightBackTargetDist){

        targetDist[0] = leftFrontTargetDist;
        targetDist[1] = leftBackTargetDist;
        targetDist[2] = rightFrontTargetDist;
        targetDist[3] = rightBackTargetDist;

        zeroPos[0] = leftFront.getCurrentPosition();
        zeroPos[1] = leftBack.getCurrentPosition();
        zeroPos[2] = rightFront.getCurrentPosition();
        zeroPos[3] = rightBack.getCurrentPosition();

        targetClicks[0] = (int)(targetDist[0] * TICKS_PER_INCH);
        targetClicks[1] = (int)(targetDist[1] * TICKS_PER_INCH);;
        targetClicks[2] = (int)(targetDist[2] * TICKS_PER_INCH);;
        targetClicks[3]= (int)(targetDist[3] * TICKS_PER_INCH);;

        leftFront.setPower(.1);
        leftBack.setPower(-.14);
        rightFront.setPower(.1);
        rightBack.setPower(-.14);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//setting mode to run using encoder
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//might be useless code, saw it used in someone else's so its here
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//gonna check it out later
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        float initAngle=angles.firstAngle;//find the heading of the robot before moving, trying to preserve this value throughout the motion
        float deltaAngle=0;//the angle that we deviate
        int correctionCompensation=0;//This variable makes up for the motors motion during correction. Because leftFront will be used in
        // correction, its encoder vals will be messed up, making the while loop end early, so we
        //add enc vals to its current position so it'll end at the right time
        while (opModeIsActive() && leftFront.getCurrentPosition()-correctionCompensation<zeroPos[0]+targetClicks[0]) {
            deltaAngle=angles.firstAngle-initAngle;
            if(deltaAngle>5){
                try {
                    spinTurnUsingDOF(deltaAngle, moveDirection.RIGHT);
                    correctionCompensation+=12*Math.PI *(2*deltaAngle/360);
                }catch(Exception e){
                    e.printStackTrace();
                }
            }
        }
        brake();

    }

    public void turnRight(double degrees){
        double dist = 12*Math.PI *(2*degrees/360);
        double leftFrontTargetDist = -dist;
        double leftBackTargetDist = -dist;
        double rightFrontTargetDist = dist;
        double rightBackTargetDist = dist;

        targetDist[0] = leftFrontTargetDist;
        targetDist[1] = leftBackTargetDist;
        targetDist[2] = rightFrontTargetDist;
        targetDist[3] = rightBackTargetDist;

        zeroPos[0] = leftFront.getCurrentPosition();
        zeroPos[1] = leftBack.getCurrentPosition();
        zeroPos[2] = rightFront.getCurrentPosition();
        zeroPos[3] = rightBack.getCurrentPosition();

        targetClicks[0] = (int)(targetDist[0] * TICKS_PER_INCH);
        targetClicks[1] = (int)((int)(targetDist[1] * TICKS_PER_INCH));
        targetClicks[2] = (int)((int)(targetDist[2] * TICKS_PER_INCH));
        targetClicks[3]= (int)((int)(targetDist[3] * TICKS_PER_INCH));

        leftFront.setPower(-.1);
        leftBack.setPower(-.1);
        rightFront.setPower(.1);
        rightBack.setPower(.1);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        while(opModeIsActive() && leftFront.getCurrentPosition()<zeroPos[0]-targetClicks[0] &&
                leftBack.getCurrentPosition()<zeroPos[1]-targetClicks[1] &&
                rightFront.getCurrentPosition()<zeroPos[2]+targetClicks[2] &&
                rightBack.getCurrentPosition()<zeroPos[3]+targetClicks[3]){

            telemetry.addData("Left front pos: ", leftFront.getCurrentPosition());
            telemetry.addData("Left back pos: ", leftBack.getCurrentPosition());
            telemetry.addData("Right front pos: ", leftFront.getCurrentPosition());
            telemetry.addData("Right back pos: ", leftFront.getCurrentPosition());
            telemetry.update();

        }

    }
    public void turnLeft(double degrees){
        double dist = 12*Math.PI *(2*degrees/360);
        double leftFrontTargetDist = dist;
        double leftBackTargetDist = dist;
        double rightFrontTargetDist = -dist;
        double rightBackTargetDist = -dist;

        targetDist[0] = leftFrontTargetDist;
        targetDist[1] = leftBackTargetDist;
        targetDist[2] = rightFrontTargetDist;
        targetDist[3] = rightBackTargetDist;

        zeroPos[0] = leftFront.getCurrentPosition();
        zeroPos[1] = leftBack.getCurrentPosition();
        zeroPos[2] = rightFront.getCurrentPosition();
        zeroPos[3] = rightBack.getCurrentPosition();

        targetClicks[0] = (int)(targetDist[0] * TICKS_PER_INCH);
        targetClicks[1] = (int)((int)(targetDist[1] * TICKS_PER_INCH));
        targetClicks[2] = (int)((int)(targetDist[2] * TICKS_PER_INCH));
        targetClicks[3]= (int)((int)(targetDist[3] * TICKS_PER_INCH));

        leftFront.setPower(.1);
        leftBack.setPower(.1);
        rightFront.setPower(-.1);
        rightBack.setPower(-.1);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        while(opModeIsActive() && leftFront.getCurrentPosition()<zeroPos[0]+targetClicks[0] &&
                leftBack.getCurrentPosition()<zeroPos[1]+targetClicks[1] &&
                rightFront.getCurrentPosition()<zeroPos[2]-targetClicks[2] &&
                rightBack.getCurrentPosition()<zeroPos[3]-targetClicks[3]){

            telemetry.addData("Left front pos: ", leftFront.getCurrentPosition());
            telemetry.addData("Left back pos: ", leftBack.getCurrentPosition());
            telemetry.addData("Right front pos: ", leftFront.getCurrentPosition());
            telemetry.addData("Right back pos: ", leftFront.getCurrentPosition());
            telemetry.update();

        }

    }
    public void moveDistBack(int leftFrontTargetDist, int leftBackTargetDist, int rightFrontTargetDist, int rightBackTargetDist){

        targetDist[0] = leftFrontTargetDist;
        targetDist[1] = leftBackTargetDist;
        targetDist[2] = rightFrontTargetDist;
        targetDist[3] = rightBackTargetDist;

        zeroPos[0] = leftFront.getCurrentPosition();
        zeroPos[1] = leftBack.getCurrentPosition();
        zeroPos[2] = rightFront.getCurrentPosition();
        zeroPos[3] = rightBack.getCurrentPosition();

        targetClicks[0] = (int)(targetDist[0] * TICKS_PER_INCH);
        targetClicks[1] = (int)(targetDist[1] * TICKS_PER_INCH);;
        targetClicks[2] = (int)(targetDist[2] * TICKS_PER_INCH);;
        targetClicks[3]= (int)(targetDist[3] * TICKS_PER_INCH);;

        leftFront.setPower(-.1);
        leftBack.setPower(-.1);
        rightFront.setPower(-.1);
        rightBack.setPower(-.1);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        while(opModeIsActive() && leftFront.getCurrentPosition()>zeroPos[0]-targetClicks[0] &&
                leftBack.getCurrentPosition()>zeroPos[1]-targetClicks[1] &&
                rightFront.getCurrentPosition()>zeroPos[2]-targetClicks[2] &&
                rightBack.getCurrentPosition()>zeroPos[3]-targetClicks[3]){

            telemetry.addData("Left front pos: ", leftFront.getCurrentPosition());
            telemetry.addData("Left back pos: ", leftBack.getCurrentPosition());
            telemetry.addData("Right front pos: ", leftFront.getCurrentPosition());
            telemetry.addData("Right back pos: ", leftFront.getCurrentPosition());
            telemetry.update();

        }

    }
    public void moveDistForward(int leftFrontTargetDist, int leftBackTargetDist, int rightFrontTargetDist, int rightBackTargetDist){

        targetDist[0] = leftFrontTargetDist;
        targetDist[1] = leftBackTargetDist;
        targetDist[2] = rightFrontTargetDist;
        targetDist[3] = rightBackTargetDist;

        zeroPos[0] = leftFront.getCurrentPosition();
        zeroPos[1] = leftBack.getCurrentPosition();
        zeroPos[2] = rightFront.getCurrentPosition();
        zeroPos[3] = rightBack.getCurrentPosition();

        targetClicks[0] = (int)(targetDist[0] * TICKS_PER_INCH);
        targetClicks[1] = (int)(targetDist[1] * TICKS_PER_INCH);;
        targetClicks[2] = (int)(targetDist[2] * TICKS_PER_INCH);;
        targetClicks[3]= (int)(targetDist[3] * TICKS_PER_INCH);;

        leftFront.setPower(.1);
        leftBack.setPower(.1);
        rightFront.setPower(.1);
        rightBack.setPower(.1);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        while(opModeIsActive() && leftFront.getCurrentPosition()<zeroPos[0]+targetClicks[0] &&
                leftBack.getCurrentPosition()<zeroPos[1]+targetClicks[1] &&
                rightFront.getCurrentPosition()<zeroPos[2]+targetClicks[2] &&
                rightBack.getCurrentPosition()<zeroPos[3]+targetClicks[3]){

            telemetry.addData("Left front pos: ", leftFront.getCurrentPosition());
            telemetry.addData("Left back pos: ", leftBack.getCurrentPosition());
            telemetry.addData("Right front pos: ", leftFront.getCurrentPosition());
            telemetry.addData("Right back pos: ", leftFront.getCurrentPosition());
            telemetry.update();

        }

    }


    public void brake(){
        leftFront.setPower(0.01);
        leftBack.setPower(-0.01);
        rightFront.setPower(0.01);
        rightBack.setPower(-0.01);
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}
