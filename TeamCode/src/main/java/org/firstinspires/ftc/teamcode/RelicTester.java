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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="WORLDSRelicTester", group="Iterative Opmode")

public class RelicTester extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();


    private Servo relicGrabber = null;
    private Servo relicClaw = null;

    private boolean grabberUp = true;
    private boolean clawClosed = true;

    private double grabberunpressed = 0.0;
    private double clawunpressed = 0.0;

    private double GRABBERUP = 1.0;
    private double GRABBERDOWN = 0.29;
    private double GRABBERINIT = 0.0;
    private double CLAWCLOSE = 0.85;
    private double CLAWOPEN = 0.35;
    private double CLAWINIT = 0.35;

    static final double LEFT_SHOULDER_IN = 0.15;
    static final double LEFT_SHOULDER_OUT = 0.60;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initialized");


        relicClaw = hardwareMap.get(Servo.class, "relicClaw");
        relicGrabber = hardwareMap.get(Servo.class, "relicGrabber");



        relicGrabber.setPosition(GRABBERINIT);
        relicClaw.setPosition(CLAWINIT);


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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


            if(gamepad2.a&&runtime.seconds()>grabberunpressed){
                if(grabberUp){
                    relicGrabber.setPosition(GRABBERDOWN);
                }
                else{
                    relicGrabber.setPosition(GRABBERUP);
                }
                grabberunpressed = runtime.seconds()+0.4;
                grabberUp = !grabberUp;
            }
            if(gamepad2.x&&runtime.seconds()>clawunpressed){
                if(clawClosed){
                    relicClaw.setPosition(CLAWOPEN);
                }
                else{
                    relicClaw.setPosition(CLAWCLOSE);
                }
                clawunpressed = runtime.seconds()+0.4;
                clawClosed = !clawClosed;
            }


            telemetry.addData("right trigger", gamepad1.right_trigger);
            telemetry.addData("left trigger", gamepad1.left_trigger);



        }



    @Override
    public void stop(){
    }


    //ALL OF THE NUMBERS WE'RE USING HERE ARE MAGIC NUMBERS, SEE THE DESMOS GRAPH FOR WHY WE USED THEM
    //OR JUST PLUG IT INTO DESMOS YOURSELF
    public static float getWheelPower(double in){

        if(in<0){// if in is negative
            in*=100;
            if(in>=-9){
                in = (float)((in*0.0315)/100);
            }
            else if(in<-9 && in>=-39.537){
                in = (float)((0.021*(Math.pow(in-2, 2))+0.063)/-1000);
            }
            else{
                in = (float)((0.75*in)/100);
            }
            return (float)(in);
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
            return (float)(in);
        }

    }
}
