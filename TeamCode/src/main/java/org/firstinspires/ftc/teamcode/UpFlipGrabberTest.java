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


@TeleOp(name = "UpFLipGrabber", group = "Iterative Opmode")
@Disabled
public class UpFlipGrabberTest extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private Servo downLeft = null;
    private Servo downRight = null;
    private Servo upLeft = null;
    private Servo upRight = null;
    private Servo rotLeft = null;
    private Servo rotRight = null;

    private double GRABBERDOWNLEFTCLOSE = 0.02;
    private double GRABBERDOWNRIGHTCLOSE = 0.93;
    private double GRABBERDOWNLEFTOPEN = 0.2;
    private double GRABBERDOWNRIGHTOPEN = 0.75;
    private double GRABBERUPLEFTCLOSE = 0.00;
    private double GRABBERUPRIGHTCLOSE = 0.98;
    private double GRABBERUPLEFTOPEN = 0.15;
    private double GRABBERUPRIGHTCOPEN = 0.8;
    private double ROTLEFTDOWN = 0.11;
    private double ROTRIGHTDOWN = 0.64;
    private double ROTLEFTUP = 0.47;
    private double ROTRIGHTUP = 0.27;

    private boolean rotUP = true;
    private double rotUnpress = 0.0;
    private boolean downClose = false;
    private double downUnpress = 0.0;
    private boolean upCLose = false;
    private double upUnpress = 0.0;



    //private float x1, x2, y1, y2;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initialized");
        downLeft = hardwareMap.servo.get("downLeft");
        downRight = hardwareMap.servo.get("downRight");
        upLeft = hardwareMap.servo.get("upLeft");
        upRight = hardwareMap.servo.get("upRight");
        rotLeft = hardwareMap.servo.get("rotLeft");
        rotRight = hardwareMap.servo.get("rotRight");

        runtime.reset();


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

        //relicGrabber.setPosition(startPosGrabber);
        //relicExtender.setPosition(startPosExtender);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if(gamepad2.x&&runtime.milliseconds()>rotUnpress){
            if(rotUP){
                rotLeft.setPosition(ROTLEFTDOWN);
                rotRight.setPosition(ROTRIGHTDOWN);
                rotUP = false;
                rotUnpress = runtime.milliseconds()+200;
            }
            else{
                rotLeft.setPosition(ROTLEFTUP);
                rotRight.setPosition(ROTRIGHTUP);
                rotUP = true;
                rotUnpress = runtime.milliseconds()+200;
            }
        }
        if(gamepad2.y&&runtime.milliseconds()>downUnpress){
            if(!downClose){
                downLeft.setPosition(GRABBERDOWNLEFTCLOSE);
                downRight.setPosition(GRABBERDOWNRIGHTCLOSE);
                downClose=true;
                downUnpress= runtime.milliseconds()+200;
            }
            else{
                downLeft.setPosition(GRABBERDOWNLEFTOPEN);
                downRight.setPosition(GRABBERUPRIGHTCOPEN);
                downClose=false;
                downUnpress = runtime.milliseconds()+200;
            }
        }
        if(gamepad2.a&&runtime.milliseconds()>upUnpress){
            if(!upCLose){
                upLeft.setPosition(GRABBERUPLEFTCLOSE);
                upRight.setPosition(GRABBERUPRIGHTCLOSE);
                upCLose = true;
                upUnpress = runtime.milliseconds()+200;
            }
            else{
                upLeft.setPosition(GRABBERUPLEFTOPEN);
                upRight.setPosition(GRABBERUPRIGHTCOPEN);
                upCLose = false;
                upUnpress = runtime.milliseconds()+200;
            }
        }
        if(gamepad2.b){
            upLeft.setPosition(GRABBERUPLEFTCLOSE);
            upRight.setPosition(GRABBERUPRIGHTCLOSE);
            downLeft.setPosition(GRABBERDOWNLEFTOPEN);
            downRight.setPosition(GRABBERDOWNRIGHTOPEN);
            rotLeft.setPosition(ROTLEFTUP);
            rotRight.setPosition(ROTRIGHTUP);
            rotUP=true;
            downClose=false;
            upCLose = true;
        }
        telemetry.addData("Grabber down: ",downClose);
        telemetry.addData("Grabber up: ",upCLose);
        telemetry.addData("Rot up: ",rotUP);
        telemetry.update();
    }

}
