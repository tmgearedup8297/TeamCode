/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.CryptoboxDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;




@Autonomous(name="Red1CV", group="Linear Opmode")

public class Red1CV extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private int pos;

    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    private CryptoboxDetector cryptoboxDetector = null;


    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    private final int TICKS_PER_INCH=36;


    private int[] zeroPos = {0, 0, 0, 0};
    private double[] targetDist = {24, 24, 24, 24};
    private int[] targetClicks = {0, 0, 0, 0};

    final int[] targetvals = {490,478,511};

    private ColorSensor jewelSensor;
    private double spos = 0.0;
    private double epos = 0.9;
    private Servo shoulder, elbow;
    private Servo glyphRightBack = null;
    private Servo glyphLeftBack = null;
    private Servo actuatorBack = null;

    //BNO055IMU imu;

    // State used for updating telemetry
    //Orientation angles;
    //Acceleration gravity;
    //private float initAngle;

    @Override
    public void runOpMode() {

        /*BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
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
        float deltaAngle = 0;*/

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        vuforiaParameters.vuforiaLicenseKey = "AVUcFCn/////AAAAGRAznpbgTUrbnvSn5Odx3WFSWKyWk+CaQKGEJCcm033tEoqUiLMTlyHFwX01tkB5QfFVbnJAp8kTS442QovsVniqOrTAxcbKJHNzbYEVtYx/4ZDyIS7Vsb+uyE2VSNs8pKEPmsZAmW9/XJid02yP7/2K6W1nJ6NpwFKdY3qLs/7qX+M4CPeCkKdnllkgjsq99xvIOncxGorzdjNIAYfIEwHds0BhKKcR3GyyflHAOx0zvTAEvu8g0g2kSTDPxemDY5vug2O0vE61mW/AL5YIIlJeWf3dfpPJg7SjP8RxS44vMoW8bpEkfJXD2KhZpigDGP+SV1JHLz2d3DrpP+TBFSQG00+F93Dd7gofT8n7rpyk";

        vuforiaParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(vuforiaParameters);


        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        cryptoboxDetector = new CryptoboxDetector();
        cryptoboxDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 1);
        cryptoboxDetector.rotateMat = false;

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        shoulder = hardwareMap.servo.get("shoulder");
        elbow = hardwareMap.servo.get("elbow");
        jewelSensor = hardwareMap.colorSensor.get("jewelSensor");
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

        actuatorBack = hardwareMap.get(Servo.class, "actuatorBack");
        glyphLeftBack = hardwareMap.servo.get("glyphLeftBack");
        glyphRightBack = hardwareMap.servo.get("glyphRightBack");
        glyphLeftBack.setDirection(Servo.Direction.REVERSE);
        glyphRightBack.setDirection(Servo.Direction.FORWARD);


        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elbow.setPosition(.9);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //shoulder.setPosition(spos);
        // elbow.setPosition(epos);

        waitForStart();

        relicTrackables.activate();
        // Wait for the game to start (driver presses PLAY)

        runtime.reset();
        //glyphRightBack.setPosition(.7);
        //glyphLeftBack.setPosition(.815);

        /*shoulder.setPosition(.42);
        sleep(1000);

        elbow.setPosition(.37);
        sleep(1000);



        sleep(1500);
       //// telemetry.addData("Blue val:", jewelSensor.blue());
        //telemetry.addData("Red val:", jewelSensor.red());
        //telemetry.update();
        if(jewelSensor.red()==0 && jewelSensor.blue()==0) {
            telemetry.addData("Can't Read", "");
            elbow.setPosition(.9);
            sleep(1000);
        }
        else if(jewelSensor.red()<jewelSensor.blue()){
            shoulder.setPosition(.6);

            sleep(1000);
           // telemetry.addData("Shoulder: ", shoulder.getPosition());

            telemetry.update();
            elbow.setPosition(.9);
            sleep(1000);
            //}
        }
        else{
            shoulder.setPosition(.25);
            sleep(1000);
            elbow.setPosition(.9);
            sleep(1000);

        }
        shoulder.setPosition(0);*/

        sleep(1000);
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        String poString = vuMark.toString();

        if(poString.equals("LEFT")){
            pos=0;
        }else if(poString.equals("CENTER")){
            pos=1;
        }else if(poString.equals("RIGHT")) {
            pos=2;
        }

        telemetry.addData("Pos", pos);
        telemetry.addData("VuMark", vuMark);
        telemetry.update();
        sleep(5000);
        telemetry.update();



        moveDistBack(30, 30, 30, 30);
        brake();
        sleep(500);
        cryptoboxDetector.enable();
        strafeDistRight(12,12,12,12);
        brake();
        sleep(2000);
        //while(!cryptoboxDetector.isCryptoBoxDetected())
            //strafeDistRight(1,1,1,1);
        //brake();
        //sleep(250);
        int[] colvals = cryptoboxDetector.getCryptoBoxPositions();
        cryptoboxDetector.disable();
        telemetry.addData("Column values: ", colvals);
        telemetry.update();

        //}*/


        // run until the end of the match (driver presses STOP)

    }

    /*public void spinTurnUsingDOF(float degrees, String direction) throws InterruptedException {
        Orientation initialOrientation = imu.getAngularOrientation();
        float angle_0 = initialOrientation.firstAngle;

        if (direction.equalsIgnoreCase("right")) {
            degrees=angle_0+degrees;
            while ((imu.getAngularOrientation().firstAngle < degrees) && opModeIsActive()) {

                leftBack.setPower(.1);
                leftFront.setPower(.1);
                rightFront.setPower(-.1);
                rightBack.setPower(-.1);

            }
            brake();
            //telemetry.addData("DOF gyro value WHILE RIGHT", realOrientation);
            telemetry.update();
        } else if (direction.equalsIgnoreCase("left")) {

            while ((imu.getAngularOrientation().firstAngle > degrees) && opModeIsActive()) {
                leftBack.setPower(-0.1);
                leftFront.setPower(-0.1);
                rightFront.setPower(0.1);
                rightBack.setPower(0.1);
            }
            brake();
            telemetry.update();
        }
        brake();

    }*/
    public void strafeDistRight(int leftFrontTargetDist, int leftBackTargetDist, int rightFrontTargetDist, int rightBackTargetDist) {
        //initAngle = angles.firstAngle;

        targetDist[0] = leftFrontTargetDist;
        targetDist[1] = leftBackTargetDist;
        targetDist[2] = rightFrontTargetDist;
        targetDist[3] = rightBackTargetDist;

        zeroPos[0] = leftFront.getCurrentPosition();
        zeroPos[1] = leftBack.getCurrentPosition();
        zeroPos[2] = rightFront.getCurrentPosition();
        zeroPos[3] = rightBack.getCurrentPosition();

        targetClicks[0] = (int) (targetDist[0] * TICKS_PER_INCH);
        targetClicks[1] = (int) (targetDist[1] * TICKS_PER_INCH);
        targetClicks[2] = (int) (targetDist[2] * TICKS_PER_INCH);
        targetClicks[3] = (int) (targetDist[3] * TICKS_PER_INCH);

        //THIS IS GOOD CODDDDDDDDDeeee
        leftFront.setPower(.14);
        leftBack.setPower(-.1);
        rightFront.setPower(.1);
        rightBack.setPower(-.14);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while (opModeIsActive() && leftFront.getCurrentPosition() < zeroPos[0] + targetClicks[0] &&
                rightBack.getCurrentPosition() > zeroPos[3] - targetClicks[3]) {

            telemetry.update();
        }
        brake();

        /*float deltaAngle=initAngle-angles.firstAngle;
        try{
            spinTurnUsingDOF(-deltaAngle, "left");
            telemetry.addData("InitAngle", initAngle);
            telemetry.addData("Delta angle", deltaAngle);
            telemetry.addData("Current angle", angles.firstAngle);
            telemetry.update();
        }catch(Exception e){
            e.printStackTrace();
        }*/


    }
    public void strafeDistLeft(int leftFrontTargetDist, int leftBackTargetDist, int rightFrontTargetDist, int rightBackTargetDist){
        //initAngle = angles.firstAngle;

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

        //THIS IS GOOD CODEEEEEEEEEEEEEEEE
        leftFront.setPower(-.14);
        leftBack.setPower(.1);
        rightFront.setPower(-.14);
        rightBack.setPower(.12);



        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() && leftFront.getCurrentPosition() > zeroPos[0] - targetClicks[0] &&
                rightBack.getCurrentPosition() < zeroPos[3] + targetClicks[3]) {

            telemetry.update();
        }

       /*float deltaAngle=initAngle-angles.firstAngle;
        try{
            spinTurnUsingDOF(deltaAngle, "right");
            telemetry.addData("InitAngle", initAngle);
            telemetry.addData("Delta angle", deltaAngle);
            telemetry.addData("Current angle", angles.firstAngle);
            telemetry.update();
        }catch(Exception e){
            e.printStackTrace();
        }*/
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
