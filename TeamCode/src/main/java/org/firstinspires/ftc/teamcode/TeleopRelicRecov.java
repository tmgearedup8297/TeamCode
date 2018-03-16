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

import static android.R.attr.x;
import static android.R.attr.y;
import static android.os.SystemClock.sleep;
import static com.qualcomm.robotcore.util.Range.scale;


@TeleOp(name="TeleopRelicRecovOld", group="Iterative Opmode")
@Disabled
public class TeleopRelicRecov extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor upDownFront= null;
    private DcMotor upDownBack = null;
    private Servo glyphRightFront = null;
    private Servo glyphLeftFront = null;
    private Servo glyphRightBack = null;
    private Servo glyphLeftBack = null;
    private Servo actuatorFront = null;
    private Servo actuatorBack = null;


    private float x1, x2, y1, y2;

    //private final double SERVO_POS_CLOSED = .6;
    //private final double SERVO_POS_OPEN = .4;

    /*static final double FRONT_SERVO_EXTENDED=.75;
    static final double FRONT_SERVO_RETRACTED=.45;
    static final double BACK_SERVO_EXTENDED=.75;
    static final double BACK_SERVO_RETRACTED=.45;*/

    private boolean activatorOpen = false;
    private boolean GRABBER_OPEN=true;
    private boolean direction = false;
    private int actuatorCount=0;
    private Servo shoulder, elbow;
    private ColorSensor jewelSensor;


    private boolean rbLastPass = false;
    private boolean slowTurn = true;
    private double turnPercent = 1.0;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        shoulder = hardwareMap.servo.get("shoulder");
        elbow = hardwareMap.servo.get("elbow");
        jewelSensor = hardwareMap.colorSensor.get("jewelSensor");

        telemetry.addData("Status", "Initialized");
        upDownFront  = hardwareMap.get(DcMotor.class, "liftFront");
        upDownBack = hardwareMap.get(DcMotor.class, "liftBack");

        glyphLeftFront = hardwareMap.servo.get("glyphLeftFront");
        glyphRightFront = hardwareMap.servo.get("glyphRightFront");
        glyphLeftBack = hardwareMap.servo.get("glyphLeftBack");
        glyphRightBack = hardwareMap.servo.get("glyphRightBack");

        //glyphLeft.setDirection(Servo.Direction.REVERSE);
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        actuatorBack = hardwareMap.get(Servo.class, "actuatorBack");
        actuatorFront = hardwareMap.get(Servo.class, "actuatorFront");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        glyphLeftBack.setDirection(Servo.Direction.REVERSE);
        glyphLeftFront.setDirection(Servo.Direction.REVERSE);



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
        glyphRightBack.setPosition(.71);
        glyphLeftBack.setPosition(.825);
        //actuatorFront.setPosition(.7);
        actuatorBack.setPosition(.7);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        elbow.setPosition(.1);
        shoulder.setPosition(1);


        if(gamepad2.x){
            actuatorBack.setPosition(0.2);
        }
        if(gamepad2.y){
            actuatorBack.setPosition(0);
        }

        upDownBack.setPower(gamepad2.left_stick_y);


        if (gamepad2.a) {
            glyphRightBack.setPosition(.68);
            glyphLeftBack.setPosition(.795);
            //.68 and .815
        }
        if (gamepad2.b) {
            glyphLeftBack.setPosition(.925);
            glyphRightBack.setPosition(1);
        }




        float leftY= getWheelPower(gamepad1.left_stick_y);
        float leftX= getWheelPower(gamepad1.left_stick_x);
        float rightX= getWheelPower(gamepad1.right_stick_x);

        telemetry.addData("LeftXRaw", gamepad1.left_stick_x);
        telemetry.addData("LeftYRaw", gamepad1.left_stick_y);
        telemetry.addData("RightXRaw",gamepad1.right_stick_x);
        telemetry.addData("LeftX", leftX);
        telemetry.addData("LeftY", leftY);
        telemetry.addData("RightX", rightX);




        leftFront.setPower((leftY-leftX-rightX));
        leftBack.setPower(leftY+leftX-rightX);
        rightFront.setPower((leftY-leftX+rightX));
        rightBack.setPower((leftY+leftX+rightX));






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
