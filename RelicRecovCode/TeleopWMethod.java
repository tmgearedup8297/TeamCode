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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Arrays;
import com.qualcomm.robotcore.hardware.ColorSensor; //

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="TeleopWMethodOld", group="Iterative Opmode")
//@Disabled
public class TeleopWMethod extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    //
    //private Servo glyphLeftDown = null;
    private Servo glyphRightDown = null;

    private Servo shoulderRight = null;
    private Servo shoulderLeft = null;
    private Servo elbowRight = null;
    private Servo elbowLeft = null;
    private Servo autoGlyphLeft = null;//
    private Servo autoGlyphRight = null;
    private Servo activator = null;
    private Servo frontLimArm;

    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor lift = null;


    private float x1, x2, y1, y2;

    private final double SERVO_POS_CLOSED = .6;
    private final double SERVO_POS_OPEN = .4;

    static final double FRONT_SERVO_EXTENDED=.75;
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
    static final double ACTIVATOR_IN = 0.0;
    static final double ACTIVATOR_OUT = 0.3;
    static final double FRONTLIM_OUT = 0.13;
    static final double FRONTLIM_IN = 0.65;


    static final double LEFT_GRABBER_DOWN_CLOSE = 0.32; //good

    static final double RIGHT_GRABBER_DOWN_OPEN = 0.94;


    static final double LEFT_GRABBER_DOWN_OPEN = 0.04;

    static final double RIGHT_GRABBER_DOWN_CLOSE = 0.61;





    private DigitalChannel liftLim,frontLim;
    private ColorSensor jewelSensorRight;
    private ColorSensor jewelSensorLeft;

    //TEMPLATE FOR TOGGLE
    private boolean rbLastPass = false;
    private boolean lbLastPass = false;
    private boolean GRABBER_OPEN=true;
    private boolean activatorOpen = true;

    private int curcol = 1;
    private boolean unpressed = true;
    private int[] fill = {0,0,0};
    private double addGlyph = 0.0;


    private int tempEncoderTarget = -1;
    private int tempEncoderZero = -1;

    private boolean inMethod = false;
    private boolean brake = false;
    private boolean extendFrontServo = true;
    private boolean drivingWithinCryptoboxPower = false;
    private boolean drivingWithinCryptobox = false;
    private boolean setStrafeUntilSwitchPower = false;
    private boolean afterBrakeSection=setStrafeUntilSwitchPower;
    private boolean strafeUntilSwitch = false;
    private boolean backUpFromCryptoboxPower = false;
    private boolean backUpFromCryptobox = false;
    private boolean retractFrontServo = false;
    private boolean strafeUntilAligned = false;
    private boolean strafeUntilAlignedPower=false;

    private boolean shoulder = false;
    private boolean elbow = false;
    private boolean autoglyph = false;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {


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

        //glyphLeftDown = hardwareMap.servo.get("glyphLeftDown");
        glyphRightDown = hardwareMap.servo.get("glyphRightDown");


        shoulderLeft = hardwareMap.servo.get("shoulderLeft");
        shoulderRight = hardwareMap.servo.get("shoulderRight");
        elbowLeft = hardwareMap.servo.get("elbowLeft");
        elbowRight = hardwareMap.servo.get("elbowRight");
        jewelSensorLeft = hardwareMap.colorSensor.get("jewelSensorLeft");
        jewelSensorRight = hardwareMap.colorSensor.get("jewelSensorRight");
        autoGlyphLeft = hardwareMap.servo.get("autoGlyphLeft");
        autoGlyphRight = hardwareMap.servo.get("autoGlyphRight");
        frontLimArm = hardwareMap.servo.get("frontLimArm");

        liftLim = hardwareMap.get(DigitalChannel.class, "liftLim");
        frontLim = hardwareMap.get(DigitalChannel.class, "frontLim");

        telemetry.addData("Status", "Initialized");


        elbowLeft.setDirection((Servo.Direction.REVERSE));
        shoulderLeft.setDirection(Servo.Direction.REVERSE);



        activator = hardwareMap.get(Servo.class, "activator");
        activator.setPosition(ACTIVATOR_OUT);

        autoGlyphLeft = hardwareMap.get(Servo.class, "autoGlyphLeft");
        autoGlyphRight = hardwareMap.get(Servo.class, "autoGlyphRight");
        autoGlyphLeft.setDirection(Servo.Direction.REVERSE);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if (!inMethod) {
            inMethod = false;
            brake = false;
            extendFrontServo = true;
            drivingWithinCryptoboxPower = false;
            drivingWithinCryptobox = false;
            setStrafeUntilSwitchPower = false;
            afterBrakeSection = setStrafeUntilSwitchPower;
            strafeUntilSwitch = false;
            backUpFromCryptoboxPower = false;
            backUpFromCryptobox = false;
            retractFrontServo = false;
            strafeUntilAligned = false;
            strafeUntilAlignedPower = false;
            boolean rbPressed = gamepad2.right_bumper;
            if (rbPressed && !rbLastPass) {
                GRABBER_OPEN = !GRABBER_OPEN;
                if (!GRABBER_OPEN) {

                    glyphRightDown.setPosition(RIGHT_GRABBER_DOWN_CLOSE);

                    //glyphLeftDown.setPosition(LEFT_GRABBER_DOWN_CLOSE);

                    telemetry.addData("Closing", "");
                    //telemetry.update();
                } else {

                    glyphRightDown.setPosition(RIGHT_GRABBER_DOWN_OPEN);
                    //glyphLeftDown.setPosition(LEFT_GRABBER_DOWN_OPEN);

                    telemetry.addData("Opening", "");
                    //telemetry.update();
                }
            }
            rbLastPass = rbPressed;

            /*FAIL SAFE ACTIVATOR CODE
            if(gamepad2.left_bumper){
                activator.setPosition(ACTIVATOR_OUT);
            }
            else {
                activator.setPosition(ACTIVATOR_IN);
            }*/
            boolean lbPressed = gamepad2.left_bumper;
            if (lbPressed && !lbLastPass) {
                activatorOpen = !activatorOpen;
                if (!activatorOpen) {
                    activator.setPosition(ACTIVATOR_IN);

                    telemetry.addData("Closing", "");
                    //telemetry.update();
                } else {
                    activator.setPosition(ACTIVATOR_OUT);

                    telemetry.addData("Opening", "");
                    //telemetry.update();
                }
            }
            lbLastPass = lbPressed;


            telemetry.update();


            float leftY = getWheelPower(gamepad1.left_stick_y);
            float leftX = getWheelPower(gamepad1.left_stick_x);
            float rightX = getWheelPower(gamepad1.right_stick_x);

            leftFront.setPower((leftY - leftX - rightX));
            leftBack.setPower(leftY + leftX - rightX);
            rightFront.setPower((leftY + leftX + rightX));
            rightBack.setPower((leftY - leftX + rightX));

            lift.setPower(-gamepad2.left_stick_y);
            //Cryptobox update code
            if (gamepad2.x || gamepad2.a || gamepad2.b) {
                if (gamepad2.x)
                    curcol = 0;
                else if (gamepad2.a)
                    curcol = 1;
                else if (gamepad2.b)
                    curcol = 2;
                inMethod = true;
            }
            if (gamepad2.dpad_up)
                frontLimArm.setPosition(FRONTLIM_OUT);
            if (gamepad2.dpad_down)
                frontLimArm.setPosition(FRONTLIM_IN);
        } else if (!gamepad2.left_bumper) {



           /* if(extendFrontServo){//extends front servo, allows code to move onto the next statement
                telemetry.addData("Extending arm","");
                telemetry.update();
                frontLimArm.setPosition(FRONTLIM_OUT);
                extendFrontServo=false;
                drivingWithinCryptoboxPower=true;

            }
            else if(drivingWithinCryptoboxPower){//set power to motors and do the math to move 12 in
                telemetry.addData("Setting power to motors","");
                telemetry.update();
                leftFront.setPower(-.15);
                leftBack.setPower(-.15);
                rightFront.setPower(-.17);
                rightBack.setPower(-.17);
                int inches = 12;
                tempEncoderZero = leftFront.getCurrentPosition();
                tempEncoderTarget = inches*36;
                drivingWithinCryptobox=true;
                drivingWithinCryptoboxPower=false;
            }
            else if(drivingWithinCryptobox){//basically the while loop from the move methods
                telemetry.addData("Drive within crytobox","");
                telemetry.update();
                if(Math.abs(leftFront.getCurrentPosition()-(tempEncoderZero+tempEncoderTarget))!=0) {
                    //telemetry.addData("Driving Forward","");
                    telemetry.addData("LeftFront",tempEncoderTarget+tempEncoderTarget-leftFront.getCurrentPosition());



                    telemetry.update();

                }
                else {
                    drivingWithinCryptoboxPower = false;
                    brake = true;
                    afterBrakeSection = setStrafeUntilSwitchPower;
                }
            }
            else if(brake){//braking after moving
                telemetry.addData("brake","");
                telemetry.update();
                leftFront.setPower(0.01);
                leftBack.setPower(-0.01);
                rightFront.setPower(0.01);
                rightBack.setPower(-0.01);
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightFront.setPower(0);
                rightBack.setPower(0);
                brake = false;
                afterBrakeSection = true;
            }
            else if(setStrafeUntilSwitchPower){
                telemetry.addData("Strafing","");
                telemetry.update();
                leftFront.setPower(.18);
                leftBack.setPower(-.15);
                rightFront.setPower(-.18);
                rightBack.setPower(.15);
                setStrafeUntilSwitchPower=false;
                strafeUntilSwitch = true;
            }
            else if(strafeUntilSwitch){
                telemetry.addData("waiting for switch touch","");
                telemetry.update();
                if(!frontLim.getState()) {
                    telemetry.addData("waiting for button press", "");
                    telemetry.update();
                }
                else {
                    strafeUntilSwitch=false;
                    brake = true;
                    afterBrakeSection = backUpFromCryptoboxPower;
                }
            }
            else if(backUpFromCryptoboxPower){
                telemetry.addData("backing up","");
                telemetry.update();
                leftFront.setPower(.15);
                leftBack.setPower(.15);
                rightFront.setPower(.15);
                rightBack.setPower(.15);
                int inches = 4;
                tempEncoderTarget = 4*36;
                tempEncoderZero=leftBack.getCurrentPosition();
                backUpFromCryptoboxPower=false;
                backUpFromCryptobox = true;
            }
            else if(backUpFromCryptobox){
                telemetry.addData("backing up else if statement thingy","");
                telemetry.update();
                if(leftBack.getCurrentPosition()<(tempEncoderZero+tempEncoderTarget)){
                    telemetry.addData("Backing up","");
                    telemetry.update();
                }
                else{
                    backUpFromCryptobox=false;
                    brake=true;
                    afterBrakeSection=strafeUntilAlignedPower;
                    frontLimArm.setPosition(FRONTLIM_IN);
                }
            }
            else if(strafeUntilAlignedPower){
                telemetry.addData("aligning with cryptobox set power","");
                telemetry.update();
                leftFront.setPower(.18);
                leftBack.setPower(-.15);
                rightFront.setPower(-.18);
                rightBack.setPower(.15);

                int inches = 4;
                tempEncoderZero = leftFront.getCurrentPosition();
                tempEncoderTarget = inches*36;
                drivingWithinCryptobox=true;
                drivingWithinCryptoboxPower=false;
            }
            else if(strafeUntilAligned){
                telemetry.addData("strafing until aligned","");
                telemetry.update();
                if(leftFront.getCurrentPosition()>(tempEncoderZero-tempEncoderTarget)) {
                    telemetry.addData("Driving Forward","");
                    telemetry.update();
                }
                else {
                    drivingWithinCryptoboxPower = false;
                    brake = true;
                    afterBrakeSection = extendFrontServo;
                }
            }





        }
        else {
            inMethod = false;
        }


        //MESSING AROUND W/ AUTOSCORE
        /*double targetDistance=21;
        if(rangeSensor.getDistance(DistanceUnit.CM)>targetDistance){
            telemetry.addData("distance to move", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        else {
            telemetry.addData("Finished", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }*/


        }

    }
    @Override
    public void stop(){
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
