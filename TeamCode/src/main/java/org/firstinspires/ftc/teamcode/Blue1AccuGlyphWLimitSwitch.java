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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive TeleopPlayground for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Blue1AccuGlyphWLimitSwitch", group="Linear Opmode")
//@Disabled
public class Blue1AccuGlyphWLimitSwitch extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private int pos=1;

    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;


    private Servo glyphLeftDown = null;
    private Servo glyphRightDown = null;
    private Servo glyphLeftUp = null;
    private Servo glyphRightUp= null;
    private Servo shoulderRight = null;
    private Servo shoulderLeft = null;
    private Servo elbowRight = null;
    private Servo elbowLeft = null;
    private Servo autoGlyphLeft = null;
    private Servo autoGlyphRight = null;
    private Servo activator = null;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor lift = null;


    static final double LEFT_SHOULDER_IN = 0.12;
    static final double LEFT_SHOULDER_OUT = 0.57;
    static final double LEFT_SHOULDER_OUT_JEWEL = 0.55;
    //static final double RIGHT_SHOULDER_IN = 0.175;
    //static final double RIGHT_SHOULDER_OUT = 0.732;
    static final double LEFT_ELBOW_OUT = 0.3;
    static final double LEFT_ELBOW_MID = 0.4;
    static final double LEFT_ELBOW_IN = 0.84;
    //static final double RIGHT_ELBOW_OUT = 0.26;
    //static final double RIGHT_ELBOW_IN = 0.94;
    static final double LEFT_AUTOGLYPH_IN = 0.0;
    static final double LEFT_AUTOGLYPH_OUT = 1.0;
    static final double RIGHT_AUTOGLYPH_IN = 0.0;
    static final double RIGHT_AUTOGLYPH_OUT = 1.0;
    static final double ACTIVATOR_IN = 0.0;
    static final double ACTIVATOR_OUT = 0.65;

    static final double LEFT_GRABBER_UP_CLOSE = 0.3;
    static final double LEFT_GRABBER_DOWN_CLOSE = 0.25; //good
    static final double RIGHT_GRABBER_UP_OPEN = 0.45;
    static final double RIGHT_GRABBER_DOWN_OPEN = 0.45;

    static final double LEFT_GRABBER_UP_OPEN = 0.75;    //good
    static final double LEFT_GRABBER_DOWN_OPEN = 0.5;
    static final double RIGHT_GRABBER_UP_CLOSE = 0.75;  //good
    static final double RIGHT_GRABBER_DOWN_CLOSE = 0.7;

    private ColorSensor jewelSensorRight;
    private ColorSensor jewelSensorLeft;
    private DigitalChannel rightLim;
    private DigitalChannel leftLim;

    private final int TICKS_PER_INCH=36;


    private int[] zeroPos = {0, 0, 0, 0};
    private double[] targetDist = {24, 24, 24, 24};
    private int[] targetClicks = {0, 0, 0, 0};



    @Override
    public void runOpMode() {



        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        lift = hardwareMap.get(DcMotor.class, "lift");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        glyphLeftDown = hardwareMap.servo.get("glyphLeftDown");
        glyphRightDown = hardwareMap.servo.get("glyphRightDown");
        glyphLeftUp = hardwareMap.servo.get("glyphLeftUp");
        glyphRightUp = hardwareMap.servo.get("glyphRightUp");

        shoulderLeft = hardwareMap.servo.get("shoulderLeft");
        shoulderRight = hardwareMap.servo.get("shoulderRight");
        elbowLeft = hardwareMap.servo.get("elbowLeft");
        elbowRight = hardwareMap.servo.get("elbowRight");
        jewelSensorLeft = hardwareMap.colorSensor.get("jewelSensorLeft");
        jewelSensorRight = hardwareMap.colorSensor.get("jewelSensorRight");
        autoGlyphLeft = hardwareMap.servo.get("autoGlyphLeft");
        autoGlyphRight = hardwareMap.servo.get("autoGlyphRight");

        telemetry.addData("Status", "Initialized");


        elbowLeft.setDirection((Servo.Direction.REVERSE));
        shoulderLeft.setDirection(Servo.Direction.REVERSE);

        leftLim = hardwareMap.get(DigitalChannel.class, "leftLim");
        rightLim = hardwareMap.get(DigitalChannel.class, "rightLim");

        activator = hardwareMap.get(Servo.class, "activator");

        autoGlyphLeft = hardwareMap.get(Servo.class, "autoGlyphLeft");
        autoGlyphRight = hardwareMap.get(Servo.class, "autoGlyphRight");
        autoGlyphLeft.setDirection(Servo.Direction.REVERSE);

        autoGlyphLeft.setPosition(RIGHT_AUTOGLYPH_OUT);
        telemetry.addData("Status", "Initialized");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        vuforiaParameters.vuforiaLicenseKey = "AVUcFCn/////AAAAGRAznpbgTUrbnvSn5Odx3WFSWKyWk+CaQKGEJCcm033tEoqUiLMTlyHFwX01tkB5QfFVbnJAp8kTS442QovsVniqOrTAxcbKJHNzbYEVtYx/4ZDyIS7Vsb+uyE2VSNs8pKEPmsZAmW9/XJid02yP7/2K6W1nJ6NpwFKdY3qLs/7qX+M4CPeCkKdnllkgjsq99xvIOncxGorzdjNIAYfIEwHds0BhKKcR3GyyflHAOx0zvTAEvu8g0g2kSTDPxemDY5vug2O0vE61mW/AL5YIIlJeWf3dfpPJg7SjP8RxS44vMoW8bpEkfJXD2KhZpigDGP+SV1JHLz2d3DrpP+TBFSQG00+F93Dd7gofT8n7rpyk";

        vuforiaParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(vuforiaParameters);


        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        relicTrackables.activate();

        waitForStart();


        // Wait for the game to start (driver presses PLAY)

        runtime.reset();
        // glyphRightBack.setPosition(.7);
        //glyphLeftBack.setPosition(.815);

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

        shoulderLeft.setPosition(LEFT_SHOULDER_OUT_JEWEL);
        sleep(1000);
        elbowLeft.setPosition(LEFT_ELBOW_OUT);
        sleep(2000);







        telemetry.addData("Blue val:", jewelSensorLeft.blue());
        telemetry.addData("Red val:", jewelSensorLeft.red());
        telemetry.update();


        if(jewelSensorLeft.red()==0 && jewelSensorLeft.blue()==0) {
            telemetry.addData("Can't Read", "");

            elbowLeft.setPosition(LEFT_ELBOW_MID);
            sleep(1000);
        }
        else if(jewelSensorLeft.red()<jewelSensorLeft.blue()){
            shoulderLeft.setPosition(LEFT_SHOULDER_OUT_JEWEL+.2);

            sleep(1000);
            telemetry.addData("Shoulder: ", shoulderLeft.getPosition());
            //shoulderLeft.setPosition(LEFT_SHOULDER_OUT_JEWEL);
            //sleep(1000);
            shoulderLeft = shoulderLeft = hardwareMap.servo.get("shoulderLeft");
            telemetry.update();
            elbowLeft.setPosition(LEFT_ELBOW_MID);
            sleep(1000);
            //}
        }
        else{
            shoulderLeft.setPosition(LEFT_SHOULDER_OUT_JEWEL-.2);
            sleep(1000);
            shoulderLeft.setPosition(LEFT_SHOULDER_OUT);
            elbowLeft.setPosition(LEFT_ELBOW_MID);
            sleep(1000);
            //shoulderLeft.setPosition(LEFT_SHOULDER_OUT);
            //sleep(1000);


        }




        moveDistBack(19,19,19,19);
        brake();
        sleep(500);
        straightBack(1.9,1.9,1.9,1.9);
        brake();
        sleep(250);
        shoulderLeft.setPosition(LEFT_SHOULDER_OUT);
        shoulderLeft = hardwareMap.servo.get("shoulderLeft");

        leftFront.setPower(-.07);
        leftBack.setPower(-.07);
        rightFront.setPower(-.07);
        rightBack.setPower(-.07);
        while(!leftLim.getState()){
            telemetry.addData("unpressed","");
            telemetry.update();
        }

        brake();
        sleep(250);
        elbowLeft.setPosition(LEFT_ELBOW_IN);
        sleep(100);
        shoulderLeft.setPosition(LEFT_SHOULDER_IN);
        sleep(250);
        if(pos==0)
            moveDistBack(4.2, 4.2, 4.2, 4.2);
        else if(pos==1)
            moveDistBack(10.25,10.25,10.25,10.25);
        else
            moveDistBack(18.25,18.25,18.25,18.25);
        brake();
        sleep(500);
        strafeDistRight(3.25, 3.25, 3.25, 3.25);
        brake();
        turnLeft(45);
        brake();
        sleep(1000);
        strafeDistRight(1.9,1.9,1.9,1.9);
        brake();
        sleep(250);
        autoGlyphLeft.setPosition(RIGHT_AUTOGLYPH_IN);
        sleep(1000);
        strafeDistLeft(2.5, 2.5, 2.5, 2.5);
        moveDistForward(4,4,4,4);
        brake();
        sleep(500);
        turnRight(45);
        brake();
        sleep(500);

        strafeDistRight(10,10,10,10);
        brake();
        sleep(500);
        strafeDistLeft(4,4,4,4);
        brake();
        sleep(500);
        if(pos==0)
            moveDistBack(5,5,5,5);
        else if(pos==2)
            moveDistForward(2,2,2,2);
        brake();
        //}


        // run until the end of the match (driver presses STOP)

    }

    public void straightBack(double leftFrontTargetDist, double leftBackTargetDist, double rightFrontTargetDist, double rightBackTargetDist) {

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

    public void strafeDistRight(double leftFrontTargetDist, double leftBackTargetDist, double rightFrontTargetDist, double rightBackTargetDist) {


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
        /*leftFront.setPower(.14);
        leftBack.setPower(-.1);
        rightFront.setPower(.1);
        rightBack.setPower(-.14);*/

        leftFront.setPower(.13);
        leftBack.setPower(-.1);
        rightFront.setPower(-.13);
        rightBack.setPower(.1);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while (opModeIsActive() && leftFront.getCurrentPosition() < zeroPos[0] + targetClicks[0] &&
                rightBack.getCurrentPosition() < zeroPos[3] + targetClicks[3]) {

            telemetry.update();
        }
        brake();
    }
    public void strafeDistLeft(double leftFrontTargetDist, double leftBackTargetDist, double rightFrontTargetDist, double rightBackTargetDist){


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
        /*leftFront.setPower(-.14);
        leftBack.setPower(.1);
        rightFront.setPower(-.14);
        rightBack.setPower(.12);*/

        leftFront.setPower(-.1);
        leftBack.setPower(.1);
        rightFront.setPower(.1);
        rightBack.setPower(-.1);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() && leftFront.getCurrentPosition() > zeroPos[0] - targetClicks[0] &&
                rightBack.getCurrentPosition() > zeroPos[3] - targetClicks[3]) {

            telemetry.update();
        }


    }
    public void turnRight(double degrees){
        double dist = 12.5*Math.PI *(degrees/360);
        double leftFrontTargetDist = dist;
        double leftBackTargetDist = dist;
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




        while(opModeIsActive() && leftFront.getCurrentPosition()>zeroPos[0]-targetClicks[0] &&
                leftBack.getCurrentPosition()>zeroPos[1]-targetClicks[1] &&
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
        double dist = 12.5*Math.PI *(degrees/360);
        double leftFrontTargetDist = dist;
        double leftBackTargetDist = dist;
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

        leftFront.setPower(.1);
        leftBack.setPower(.1);
        rightFront.setPower(-.1);
        rightBack.setPower(-.1);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        while(opModeIsActive() && leftFront.getCurrentPosition()<zeroPos[0]+targetClicks[0]){

            telemetry.addData("Left front pos: ", leftFront.getCurrentPosition());
            telemetry.addData("Left back pos: ", leftBack.getCurrentPosition());
            telemetry.addData("Right front pos: ", leftFront.getCurrentPosition());
            telemetry.addData("Right back pos: ", leftFront.getCurrentPosition());
            telemetry.update();

        }

    }
    public void moveDistBack(double leftFrontTargetDist, double leftBackTargetDist, double rightFrontTargetDist, double rightBackTargetDist){

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
    public void moveDistForward(double leftFrontTargetDist, double leftBackTargetDist, double rightFrontTargetDist, double rightBackTargetDist){

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
    public void brake() {
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
