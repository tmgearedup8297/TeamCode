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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


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

@Autonomous(name="Blue1", group="Linear Opmode")
//@Disabled
public class Blue1 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();



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
    static final double RIGHT_SHOULDER_IN = 0.175;
    static final double RIGHT_SHOULDER_OUT = 0.65;
    static final double LEFT_ELBOW_OUT = 0.3;
    static final double LEFT_ELBOW_IN = 0.84;
    static final double RIGHT_ELBOW_OUT = 0.25;
    static final double RIGHT_ELBOW_IN = 0.94;
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



        activator = hardwareMap.get(Servo.class, "activator");

        autoGlyphLeft = hardwareMap.get(Servo.class, "autoGlyphLeft");
        autoGlyphRight = hardwareMap.get(Servo.class, "autoGlyphRight");
        autoGlyphLeft.setDirection(Servo.Direction.REVERSE);

        autoGlyphLeft.setPosition(RIGHT_AUTOGLYPH_OUT);
        telemetry.addData("Status", "Initialized");

        waitForStart();


        // Wait for the game to start (driver presses PLAY)

        runtime.reset();
        // glyphRightBack.setPosition(.7);
        //glyphLeftBack.setPosition(.815);

        shoulderLeft.setPosition(LEFT_SHOULDER_OUT);
        sleep(1000);
        elbowLeft.setPosition(LEFT_ELBOW_OUT);
        sleep(2000);







        telemetry.addData("Blue val:", jewelSensorLeft.blue());
        telemetry.addData("Red val:", jewelSensorLeft.red());
        telemetry.update();


        if(jewelSensorLeft.red()==0 && jewelSensorLeft.blue()==0) {
            telemetry.addData("Can't Read", "");

            elbowLeft.setPosition(LEFT_ELBOW_IN);
            sleep(1000);
        }
        else if(jewelSensorLeft.red()>jewelSensorLeft.blue()){
            shoulderLeft.setPosition(LEFT_SHOULDER_OUT+.2);

            sleep(1000);
            telemetry.addData("Shoulder: ", shoulderLeft.getPosition());
            shoulderLeft.setPosition(LEFT_SHOULDER_OUT);
            sleep(1000);
            telemetry.update();
            elbowLeft.setPosition(LEFT_ELBOW_IN);
            sleep(1000);
            //}
        }
        else{
            shoulderLeft.setPosition(LEFT_SHOULDER_OUT-.2);
            sleep(1000);
            elbowLeft.setPosition(LEFT_ELBOW_IN);
            sleep(1000);
            //shoulderLeft.setPosition(LEFT_SHOULDER_OUT);
            //sleep(1000);


        }
        shoulderLeft.setPosition(LEFT_SHOULDER_IN);


        moveDistBack(30, 30, 30, 30);
        brake();
        sleep(500);
        turnLeft(45);
        brake();
        sleep(1000);
        autoGlyphLeft.setPosition(LEFT_AUTOGLYPH_IN);
        sleep(1000);
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
        //}


        // run until the end of the match (driver presses STOP)

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

        leftFront.setPower(.1);
        leftBack.setPower(-.1);
        rightFront.setPower(-.1);
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
   /* public void turnRight(double degrees){
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

    public void strafeDistRight(int leftFrontTargetDist, int leftBackTargetDist, int rightFrontTargetDist, int rightBackTargetDist){

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

        leftFront.setPower(-.01);
        leftBack.setPower(.01);
        rightFront.setPower(.01);
        rightBack.setPower(-.01);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        while(opModeIsActive() && leftFront.getCurrentPosition()<zeroPos[0]-targetClicks[0] &&
                leftBack.getCurrentPosition()<zeroPos[1]+targetClicks[1] &&
                rightFront.getCurrentPosition()<zeroPos[2]-targetClicks[2] &&
                rightBack.getCurrentPosition()<zeroPos[3]+targetClicks[3]){

            telemetry.addData("Left front pos: ", leftFront.getCurrentPosition());
            telemetry.addData("Left back pos: ", leftBack.getCurrentPosition());
            telemetry.addData("Right front pos: ", leftFront.getCurrentPosition());
            telemetry.addData("Right back pos: ", leftFront.getCurrentPosition());
            telemetry.update();

        }

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

        leftFront.setPower(.01);
        leftBack.setPower(-.01);
        rightFront.setPower(-.01);
        rightBack.setPower(.01);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        while(opModeIsActive() && leftFront.getCurrentPosition()<zeroPos[0]+targetClicks[0] &&
                leftBack.getCurrentPosition()<zeroPos[1]-targetClicks[1] &&
                rightFront.getCurrentPosition()<zeroPos[2]+targetClicks[2] &&
                rightBack.getCurrentPosition()<zeroPos[3]-targetClicks[3]){

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
    }*/
}
