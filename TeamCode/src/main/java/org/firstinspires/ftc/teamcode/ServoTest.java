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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor; //

import static android.R.attr.hardwareAccelerated;
import static android.R.attr.x;
import static android.R.attr.y;
import static android.os.SystemClock.sleep;
import static com.qualcomm.robotcore.util.Range.scale;


@TeleOp(name="ServoTest", group="Iterative Opmode")
@Disabled
public class ServoTest extends OpMode
{
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



    private float x1, x2, y1, y2;

    private final double SERVO_POS_CLOSED = .6;
    private final double SERVO_POS_OPEN = .4;

    static final double FRONT_SERVO_EXTENDED=.75;
    static final double FRONT_SERVO_RETRACTED=.45;
    static final double BACK_SERVO_EXTENDED=.75;
    static final double BACK_SERVO_RETRACTED=.45;

    static final double LEFT_SHOULDER_IN = 0.12;
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
    static final double ACTIVATOR_OUT = 0.65;
    static final double LEFT_GRABBER_UP_OPEN = 0;
    static final double LEFT_GRABBER_DOWN_OPEN = 0;
    static final double RIGHT_GRABBER_UP_OPEN = 0;
    static final double RIGHT_GRABBER_DOWN_OPEN = 0;
    static final double LEFT_GRABBER_UP_CLOSE = 1;
    static final double LEFT_GRABBER_DOWN_CLOSE = 1;
    static final double RIGHT_GRABBER_UP_CLOSE = 1;
    static final double RIGHT_GRABBER_DOWN_CLOSE = 1;



    private boolean activatorOpen = false;
    private boolean GRABBER_OPEN=true;

    private ColorSensor jewelSensorRight;
    private ColorSensor jewelSensorLeft;

    //TEMPLATE FOR TOGGLE
    //private boolean rbLastPass = false;
    //private boolean slowTurn = true;
    //private double turnPercent = 1.0;

    private boolean shoulder = false;
    private boolean elbow = false;
    private boolean autoglyph = false;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {


        glyphLeftDown = hardwareMap.servo.get("glyphLeftDown");
        glyphRightDown = hardwareMap.servo.get("glyphRightDown");
        glyphLeftUp = hardwareMap.servo.get("glyphLeftUp");
        glyphRightUp = hardwareMap.servo.get("glyphRightUp");
        shoulderLeft = hardwareMap.servo.get("shoulderLeft");
        shoulderRight = hardwareMap.servo.get("shoulderRight");
        elbowLeft = hardwareMap.servo.get("elbowLeft");
        elbowRight = hardwareMap.servo.get("elbowRight");
        jewelSensorLeft = hardwareMap.colorSensor.get("jewelSensorLeft");
        jewelSensorLeft = hardwareMap.colorSensor.get("jewelSensorLeft");
        autoGlyphLeft = hardwareMap.servo.get("autoGlyphLeft");
        autoGlyphRight = hardwareMap.servo.get("autoGlyphRight");

        telemetry.addData("Status", "Initialized");
        //upDownFront  = hardwareMap.get(DcMotor.class, "liftFront");
        //upDownBack = hardwareMap.get(DcMotor.class, "liftBack");

        elbowLeft.setDirection((Servo.Direction.REVERSE));
        shoulderLeft.setDirection(Servo.Direction.REVERSE);
        glyphLeftDown.setDirection(Servo.Direction.REVERSE);


        activator = hardwareMap.get(Servo.class, "activator");

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
        //glyphRightFront.setPosition(.7);
        //glyphLeftFront.setPosition(.7);
        //glyphRightBack.setPosition(.71);
        //glyphLeftBack.setPosition(.825);
        //actuatorFront.setPosition(.7);
        //actuatorBack.setPosition(.7);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if(gamepad1.x&&activatorOpen){
            glyphLeftDown.setPosition(LEFT_GRABBER_DOWN_OPEN);
            glyphRightDown.setPosition(RIGHT_GRABBER_DOWN_OPEN);
            glyphLeftUp.setPosition(LEFT_GRABBER_DOWN_OPEN);
            glyphRightUp.setPosition(LEFT_GRABBER_DOWN_OPEN);

            activatorOpen=false;

            telemetry.addData("Activator: ", "In");
        }
        else if(gamepad1.x){
            activator.setPosition(ACTIVATOR_OUT);
            activatorOpen=true;
            telemetry.addData("Activator: ", "Out");
        }

        if(gamepad1.y&&autoglyph){
            autoGlyphLeft.setPosition(LEFT_AUTOGLYPH_IN);
            autoGlyphRight.setPosition(RIGHT_AUTOGLYPH_IN);
            autoglyph=false;
            telemetry.addData("AutoGlyph: ", "In");
        }
        else if(gamepad1.y){
            autoGlyphLeft.setPosition(LEFT_AUTOGLYPH_OUT);
            autoGlyphRight.setPosition(RIGHT_AUTOGLYPH_OUT);
            autoglyph=true;
            telemetry.addData("AutoGlyph: ", "Out");
        }

        if(gamepad1.a&&shoulder){
            shoulderLeft.setPosition(LEFT_SHOULDER_IN);
            shoulderRight.setPosition(RIGHT_SHOULDER_IN);
            shoulder=false;
            telemetry.addData("Shoulder: ", "In");
        }
        else if(gamepad1.a) {
            shoulder=true;
            shoulderLeft.setPosition(LEFT_SHOULDER_OUT);
            shoulderRight.setPosition(RIGHT_SHOULDER_OUT);
            telemetry.addData("Shoulder: ", "Out");
        }

        if(gamepad1.b&&elbow){
            elbowLeft.setPosition(LEFT_ELBOW_OUT);
            elbowRight.setPosition(RIGHT_ELBOW_OUT);
            elbow=false;
            telemetry.addData("Elbow: ", "Out");
        }
        else if(gamepad1.b) {
            elbow=true;
            elbowLeft.setPosition(LEFT_ELBOW_IN);
            elbowRight.setPosition(RIGHT_ELBOW_IN);
            telemetry.addData("Elbow: ", "In");
        }







        //float leftY= getWheelPower(gamepad1.left_stick_y);
        //float leftX= getWheelPower(gamepad1.left_stick_x);
        //float rightX= getWheelPower(gamepad1.right_stick_x);

        //telemetry.addData("LeftXRaw", gamepad1.left_stick_x);
        //telemetry.addData("LeftYRaw", gamepad1.left_stick_y);
        //telemetry.addData("RightXRaw",gamepad1.right_stick_x);
        //telemetry.addData("LeftX", leftX);
        //telemetry.addData("LeftY", leftY);
        //telemetry.addData("RightX", rightX);




        //leftFront.setPower((leftY-leftX-rightX));
        //leftBack.setPower(leftY+leftX-rightX);
        //rightFront.setPower((leftY-leftX+rightX));
        //rightBack.setPower((leftY+leftX+rightX));






        telemetry.update();
    }


    @Override
    public void stop() {
    }

    //ALL OF THE NUMBERS WE'RE USING HERE ARE MAGIC NUMBERS, SEE THE DESMOS GRAPH FOR WHY WE USED THEM
    //OR JUST PLUG IT INTO DESMOS YOURSELF
    public static float getWheelPower(float in){

        if(in<0){// if in is negative
            in*=100;
            if(in>=-2){
                in = (float)((in*0.0315)/100);
            }
            else if(in<-2 && in>=-39.537){
                in = (float)((0.021*(Math.pow(in-2, 2))+0.063)/-100);
            }
            else{
                in = (float)((0.75*in)/100);
            }
            return in;
        }
        else{//if in is positive
            in*=100;
            if(in<=2){
                in = (float)((in*0.0315)/100);
            }
            else if(in>2 && in<=39.537){
                in = (float)((0.021*(Math.pow(in-2, 2))+0.063)/1000);
            }
            else{
                in = (float)((0.75*in)/100);
            }
            return in;
        }

    }

}
