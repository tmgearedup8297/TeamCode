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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="TeleopWMethodPlayground", group="Iterative Opmode")
//@Disabled
public class TeleopWMethodPlayground extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    //private Servo glyphLeftDown = null;
    private Servo glyphRightDown = null;

    private Servo shoulderRight = null;
    private Servo shoulderLeft = null;
    private Servo elbowRight = null;
    private Servo elbowLeft = null;
    private Servo autoGlyphLeft = null;//
    private Servo autoGlyphRight = null;
    private Servo activator = null;
    private Servo frontArm = null;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor lift = null;
    private DcMotor extender = null;
    private Servo relicGrabber=null;
    private Servo relicExtender=null;

    private static final double startPosGrabber = 0.04275;
    private static final double endPosGrabber = 0.07125;
    private static final double startPosExtender = 0.105;
    private static final double endPosExtender = 0.039;
    private static double startPosGrabberTemp = endPosExtender;

    private float x1, x2, y1, y2;

    private final double SERVO_POS_CLOSED = .6;
    private final double SERVO_POS_OPEN = .4;

    static final double FRONT_SERVO_EXTENDED=.27;
    static final double FRONT_SERVO_RETRACTED=.45;
    static final double BACK_SERVO_EXTENDED=.75;
    static final double BACK_SERVO_RETRACTED=.45;

    static final double LEFT_SHOULDER_IN = 0.09;
    static final double LEFT_SHOULDER_OUT = 0.57;
    static final double RIGHT_SHOULDER_IN = 0.175;
    static final double RIGHT_SHOULDER_OUT = 0.65;
    static final double LEFT_ELBOW_OUT = 0.3;
    static final double LEFT_ELBOW_IN = 0.84;
    static final double RIGHT_ELBOW_OUT = 0.4;
    static final double RIGHT_ELBOW_IN = 0.94;
    static final double LEFT_AUTOGLYPH_IN = 0.0;
    static final double LEFT_AUTOGLYPH_OUT = 1.0;
    static final double RIGHT_AUTOGLYPH_IN = 0.0;
    static final double RIGHT_AUTOGLYPH_OUT = 1.0;
    static final double ACTIVATOR_IN = 0.3;
    static final double ACTIVATOR_OUT = 0.7;
    static final double FRONTLIM_OUT = 0.13;
    static final double FRONTLIM_IN = 0.65;


    static final double LEFT_GRABBER_DOWN_CLOSE = 0.32; //good

    static final double RIGHT_GRABBER_DOWN_OPEN = 0.935;


    static final double LEFT_GRABBER_DOWN_OPEN = 0.04;

    static final double RIGHT_GRABBER_DOWN_CLOSE = 0.61;
    static final int TICKS_PER_INCH=36;

    private boolean firstRelic=true;

    private DigitalChannel liftLim;
    private DigitalChannel frontLim;
    private ColorSensor jewelSensorRight;
    private ColorSensor jewelSensorLeft;

    //TEMPLATE FOR TOGGLE
    private boolean rbLastPass = false;
    private boolean lbLastPass = false;
    private boolean dpadLastPass=false;
    private boolean dpadUpLastPass = false;
    private boolean relicClawOpen = false;
    private boolean dpadDownLastPass = false;
    private boolean relicClawUp = false;
    private boolean GRABBER_OPEN=true;
    private boolean activatorOpen = true;


    private double extenderPos = 0;
    private double grabberPos=0;

    private int curcol = 1;
    private boolean unpressed = true;
    private int[] fill = {0,0,0};
    private double addGlyph = 0.0;



    private boolean shoulder = false;
    private boolean elbow = false;
    private boolean autoglyph = false;

    private double tempTargetTime = -1.0;

    private boolean inMethod=false;
    private boolean extendServo=true;
    private boolean moveForward=false;
    private boolean strafeTillLim = false;
    private double initialStrafeRCClicksLeft;
    private double initialStrafeRCClicksRight;
    private double initialStrafeLCClicksLeft;
    private double initialStrafeLCClicksRight;

    private double firstMoveClicksLeft;
    private double firstMoveClicksRight;
    private int pos=1;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {


        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift = hardwareMap.get(DcMotor.class, "lift");
        extender = hardwareMap.get(DcMotor.class, "extender");

        relicExtender = hardwareMap.servo.get("relicExtender");
        relicGrabber = hardwareMap.servo.get("relicGrabber");
        frontArm = hardwareMap.servo.get("frontLimArm");
        runtime.reset();



        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);
        extender.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);


        // = hardwareMap.servo.get("glyphLeftDown");
        glyphRightDown = hardwareMap.servo.get("glyphRightDown");


        shoulderLeft = hardwareMap.servo.get("shoulderLeft");
        shoulderRight = hardwareMap.servo.get("shoulderRight");
        elbowLeft = hardwareMap.servo.get("elbowLeft");
        elbowRight = hardwareMap.servo.get("elbowRight");
        jewelSensorLeft = hardwareMap.colorSensor.get("jewelSensorLeft");
        jewelSensorRight = hardwareMap.colorSensor.get("jewelSensorRight");
        autoGlyphLeft = hardwareMap.servo.get("autoGlyphLeft");
        autoGlyphRight = hardwareMap.servo.get("autoGlyphRight");

        liftLim = hardwareMap.get(DigitalChannel.class, "liftLim");
        frontLim = hardwareMap.get(DigitalChannel.class, "frontLim");




        elbowLeft.setDirection((Servo.Direction.REVERSE));
        shoulderLeft.setDirection(Servo.Direction.REVERSE);



        activator = hardwareMap.get(Servo.class, "activator");


        autoGlyphLeft = hardwareMap.get(Servo.class, "autoGlyphLeft");
        autoGlyphRight = hardwareMap.get(Servo.class, "autoGlyphRight");
        autoGlyphLeft.setDirection(Servo.Direction.REVERSE);


        initialStrafeLCClicksLeft = leftFront.getCurrentPosition() - (9*36);
        initialStrafeLCClicksRight = rightFront.getCurrentPosition() + (9*36);

        initialStrafeRCClicksLeft = leftFront.getCurrentPosition() + (9*36);
        initialStrafeRCClicksRight = rightFront.getCurrentPosition() - (9*36);

        firstMoveClicksLeft= leftFront.getCurrentPosition() - (10*36);
        firstMoveClicksRight= rightFront.getCurrentPosition() - (10*36);


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        runtime.reset();
        autoGlyphLeft.setPosition(LEFT_AUTOGLYPH_IN);
        autoGlyphRight.setPosition(RIGHT_AUTOGLYPH_IN);
        elbowLeft.setPosition(LEFT_ELBOW_IN);
        elbowRight.setPosition(RIGHT_ELBOW_IN);
        shoulderLeft.setPosition(LEFT_SHOULDER_IN);
        shoulderRight.setPosition(RIGHT_SHOULDER_IN);
        glyphRightDown.setPosition(RIGHT_GRABBER_DOWN_CLOSE);
        activator.setPosition(ACTIVATOR_OUT);

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if(inMethod==false) {


            boolean rbPressed = gamepad2.right_bumper;
            if (rbPressed && !rbLastPass) {
                GRABBER_OPEN = !GRABBER_OPEN;
                if (!GRABBER_OPEN) {

                    glyphRightDown.setPosition(RIGHT_GRABBER_DOWN_CLOSE);

                    //.setPosition(LEFT_GRABBER_DOWN_CLOSE);


                    //telemetry.update();
                } else {

                    glyphRightDown.setPosition(RIGHT_GRABBER_DOWN_OPEN);
                    //glyphLeftDown.setPosition(LEFT_GRABBER_DOWN_OPEN);

                }
            }
            rbLastPass = rbPressed;



            if (gamepad2.a) {
                relicExtender.setPosition(endPosExtender);
                startPosGrabberTemp = endPosExtender;

            }
            if (gamepad2.x) {
                relicExtender.setPosition(startPosExtender);

            }
            if (gamepad2.b) {
                relicGrabber.setPosition(endPosGrabber);

            }
            if (gamepad2.y) {
                relicGrabber.setPosition(startPosGrabber);

            }
            if (gamepad2.dpad_up && startPosGrabberTemp < 0.062) {
                startPosGrabberTemp += 0.0007;
                relicExtender.setPosition(startPosGrabberTemp);
            }
            if (gamepad2.dpad_down && startPosGrabberTemp > 0.03) {
                startPosGrabberTemp -= 0.0007;
                relicExtender.setPosition(startPosGrabberTemp);
            }

            rbLastPass = rbPressed;


            boolean lbPressed = gamepad2.left_bumper;
            if (lbPressed && !lbLastPass) {
                activatorOpen = !activatorOpen;
                if (!activatorOpen) {
                    activator.setPosition(ACTIVATOR_IN);


                    //telemetry.update();
                } else {
                    activator.setPosition(ACTIVATOR_OUT);


                    //telemetry.update();
                }
            }
            lbLastPass = lbPressed;


            //telemetry.update();


            float leftY = getWheelPower(gamepad1.left_stick_y);
            float leftX = getWheelPower(gamepad1.left_stick_x);
            float rightX = getWheelPower(gamepad1.right_stick_x);

            leftFront.setPower((leftY - leftX - rightX));
            leftBack.setPower(leftY + leftX - rightX);
            rightFront.setPower((leftY + leftX + rightX));
            rightBack.setPower((leftY - leftX + rightX));

            lift.setPower(-gamepad2.left_stick_y);
            extender.setPower(gamepad2.right_stick_y);

            if(gamepad1.a){
                pos=1;
            }
            if(gamepad1.x){
                pos=0;
            }
            if(gamepad1.b){
                pos=2;
            }

        }
        else{
            if(init){
                if(pos==0) {
                    leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    leftFront.setPower(-.1);
                    leftBack.setPower(.1);
                    rightFront.setPower(.1);
                    rightBack.setPower(-.1);


                    leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                    if (leftFront.getCurrentPosition() > initialStrafeLCClicksLeft &&
                            rightFront.getCurrentPosition() < initialStrafeLCClicksRight) {
                        telemetry.addData("Left front pos: ", leftFront.getCurrentPosition());
                        telemetry.addData("Left back pos: ", leftBack.getCurrentPosition());
                        telemetry.addData("Right front pos: ", rightFront.getCurrentPosition());
                        telemetry.addData("Right back pos: ", rightBack.getCurrentPosition());

                        telemetry.addData("Left front left: ", firstMoveClicksLeft);
                        telemetry.addData("Right front left: ", firstMoveClicksRight);

                        telemetry.update();

                    } else {
                        leftFront.setPower(0.01);
                        leftBack.setPower(-0.01);
                        rightFront.setPower(0.01);
                        rightBack.setPower(-0.01);
                        leftFront.setPower(0);
                        leftBack.setPower(0);
                        rightFront.setPower(0);
                        rightBack.setPower(0);
                        init = false;
                        moveForward = true;
                    }
                }
                if(pos==1) {
                    leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    leftFront.setPower(-.1);
                    leftBack.setPower(.1);
                    rightFront.setPower(.1);
                    rightBack.setPower(-.1);


                    leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                    if (leftFront.getCurrentPosition() > initialStrafeLCClicksLeft &&
                            rightFront.getCurrentPosition() < initialStrafeLCClicksRight) {
                        telemetry.addData("Left front pos: ", leftFront.getCurrentPosition());
                        telemetry.addData("Left back pos: ", leftBack.getCurrentPosition());
                        telemetry.addData("Right front pos: ", rightFront.getCurrentPosition());
                        telemetry.addData("Right back pos: ", rightBack.getCurrentPosition());

                        telemetry.addData("Left front left: ", firstMoveClicksLeft);
                        telemetry.addData("Right front left: ", firstMoveClicksRight);

                        telemetry.update();

                    } else {
                        leftFront.setPower(0.01);
                        leftBack.setPower(-0.01);
                        rightFront.setPower(0.01);
                        rightBack.setPower(-0.01);
                        leftFront.setPower(0);
                        leftBack.setPower(0);
                        rightFront.setPower(0);
                        rightBack.setPower(0);
                        init = false;
                        moveForward = true;
                    }
                }
            }
            if(extendServo){
                frontArm.setPosition(FRONT_SERVO_EXTENDED);
                extendServo=false;
                moveForward=true;
            }
            else if(moveForward){



                leftFront.setPower(-.1);
                leftBack.setPower(-.1);
                rightFront.setPower(-.115);
                rightBack.setPower(-.115);

                leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




                if(leftFront.getCurrentPosition()>firstMoveClicksLeft &&
                        rightFront.getCurrentPosition()>firstMoveClicksRight){
                    telemetry.addData("Left front pos: ", leftFront.getCurrentPosition());
                    telemetry.addData("Left back pos: ", leftBack.getCurrentPosition());
                    telemetry.addData("Right front pos: ", rightFront.getCurrentPosition());
                    telemetry.addData("Right back pos: ", rightBack.getCurrentPosition());

                    telemetry.addData("Left front left: ", firstMoveClicksLeft);
                    telemetry.addData("Right front left: ", firstMoveClicksRight);

                    telemetry.update();

                }
                else{
                    leftFront.setPower(0.01);
                    leftBack.setPower(-0.01);
                    rightFront.setPower(0.01);
                    rightBack.setPower(-0.01);
                    leftFront.setPower(0);
                    leftBack.setPower(0);
                    rightFront.setPower(0);
                    rightBack.setPower(0);
                    moveForward=false;
                    strafeTillLim=true;
                }

            }
            if(strafeTillLim){
                if(frontLim.getState()==false){
                    leftFront.setPower(-.1);
                    leftBack.setPower(.1);
                    rightFront.setPower(.1);
                    rightBack.setPower(-.1);
                }
                else{
                    leftFront.setPower(-0.01);
                    leftBack.setPower(0.01);
                    rightFront.setPower(-0.01);
                    rightBack.setPower(0.01);
                    leftFront.setPower(0);
                    leftBack.setPower(0);
                    rightFront.setPower(0);
                    rightBack.setPower(0);
                    strafeTillLim=false;
                    inMethod=false;
                }
            }
        }

        if(gamepad1.a){
            inMethod=true;
        }
        if(gamepad1.b){
            inMethod=false;
        }
       // telemetry.addData("Method",inMethod);
       // telemetry.update();

    }


    @Override
    public void stop() {
    }

    //ALL OF THE NUMBERS WE'RE USING HERE ARE MAGIC NUMBERS, SEE THE DESMOS GRAPH FOR WHY WE USED THEM
    //OR JUST PLUG IT INTO DESMOS YOURSELF
    public static float getWheelPower(float in){

        if(in<0){// if in is negative
            in*=100;
            if(in>=-9){
                in = (float)((in*0.0315)/100);
            }
            else if(in<-9 && in>=-39.537){
                in = (float)((0.021*(Math.pow(in-2, 2))+0.063)/-100);
            }
            else{
                in = (float)((0.75*in)/100);
            }
            return in;
        }
        else{//if in is positive
            in*=100;
            if(in<=9){
                in = (float)((in*0.0315)/100);
            }
            else if(in>9 && in<=39.537){
                in = (float)((0.021*(Math.pow(in-2, 2))+0.063)/1000);
            }
            else{
                in = (float)((0.75*in)/100);
            }
            return in;
        }

    }
}
