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

import static android.os.SystemClock.sleep;
import static com.qualcomm.robotcore.util.Range.scale;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleopRelicRecov", group="Iterative Opmode")
//@Disabled
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
        glyphRightBack.setPosition(.7);
        glyphLeftBack.setPosition(.815);
        //actuatorFront.setPosition(.7);
        actuatorBack.setPosition(.7);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        shoulder.setPosition(0);
        elbow.setPosition(.9);

        if(gamepad2.x){
            actuatorBack.setPosition(0.2);
        }
        if(gamepad2.y){
            actuatorBack.setPosition(.7);
        }

        upDownBack.setPower(gamepad2.left_stick_y);


            if (gamepad2.a) {
                glyphLeftBack.setPosition(.68);
                glyphRightBack.setPosition(.515);
                //.68 and .815
            }
            if (gamepad2.b) {
                glyphLeftBack.setPosition(.925);
                glyphRightBack.setPosition(1);
            }


        /*else if(direction==false){
            upDownBack.setPower(gamepad2.left_stick_y);

                if (gamepad2.a) {
                    GRABBER_OPEN = true;

                } else if (gamepad2.b) {
                    GRABBER_OPEN = false;
                }

            if(GRABBER_OPEN==true){
                glyphLeftBack.setPosition(.68);
                glyphRightBack.setPosition(.515);
            }
            if(GRABBER_OPEN==false){
                glyphLeftBack.setPosition(.9);
                glyphRightBack.setPosition(.);
            }
        }*/


        /*if(gamepad2.b && activatorOpen==true){

            actuatorBack.setPosition(0);
            actuatorBack.setPosition(0);

        }
        else if(gamepad2.b && activatorOpen==false){

            actuatorBack.setPosition(.7);
            actuatorBack.setPosition(.7);

        }*/

        float leftY=(float)(gamepad1.left_stick_y *.75);
        float leftX=(float)(gamepad1.left_stick_x * .75);
        float rightX= (float)(gamepad1.right_stick_x * .75);



        leftFront.setPower((leftY-leftX-rightX));
        leftBack.setPower(leftY+leftX-rightX);
        rightFront.setPower((leftY-leftX+rightX));
        rightBack.setPower((leftY+leftX+rightX));

        if(direction==true){
            telemetry.addData("Direction", "backwards");
        }
        else{
            telemetry.addData("Direction", "forwards");
        }
        telemetry.addData("LeftBack", glyphLeftBack.getPosition());
        telemetry.addData("LeftFront", glyphLeftFront.getPosition());
        telemetry.addData("RightBack", glyphRightBack.getPosition());
        telemetry.addData("RightFront", glyphRightBack.getPosition());


        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
