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


@TeleOp(name = "TeleopRelicRecovtest", group = "Iterative Opmode")
//@Disabled
public class TeleopRelicRecovtest extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private Servo relicGrabber = null;
    private Servo relicExtender = null;
    private DcMotor Extender = null;
    static final double startPosGrabber = 0.0;
    static final double endPosGrabber = 0.04;
    static final double startPosExtender = 0.0;
    static final double endPosExtender = 0.08;
    static final double maxExtendedPos = 1.00;

    //private float x1, x2, y1, y2;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initialized");
        relicExtender = hardwareMap.servo.get("relicExtender");
        relicGrabber = hardwareMap.servo.get("relicGrabber");
        Extender= hardwareMap.dcMotor.get("Extender");

        runtime.reset();

        relicGrabber.setPosition(startPosGrabber);
        relicExtender.setPosition(startPosExtender);

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

        if (gamepad2.x) {
            relicGrabber.setPosition(startPosGrabber);
        }
        else if (gamepad2.y) {
            relicGrabber.setPosition(endPosGrabber);
        }

        if (gamepad2.a) {
            relicExtender.setPosition(startPosExtender);

        }
        else if (gamepad2.b) {
            relicExtender.setPosition(endPosExtender);
        }
        if (gamepad1.left_stick_y >= 0 && Extender.getCurrentPosition() < maxExtendedPos) {
            //Trying to extend arm out
            Extender.setPower(gamepad1.left_stick_y);
        } else if (gamepad1.left_stick_y < 0 && Extender.getCurrentPosition() >= 0) {
            //Trying to retract arm
            Extender.setPower(gamepad1.left_stick_y);
        }
        telemetry.addData("LeftYRaw", gamepad1.left_stick_y);
        telemetry.addData("motorFeedback", Extender.getCurrentPosition());
        telemetry.addData("grabberservoFeedback", relicGrabber.getPosition());
        telemetry.addData("extenderservoFeedback", relicExtender.getPosition());

        telemetry.update();

    }

}
