package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class TeleopVelVort extends MasterDeviceClass{

    //BNO055IMU dof;
    //float angle_0;


    public void giveSensorFeedback() {


        telemetry.addData(" Bottom Color ARGB", bottomcolor.argb());

 /*
        telemetry.addData("GREEN", bottomcolor.green());
        telemetry.addData("RED", bottomcolor.red());
        telemetry.addData("BLUE", bottomcolor.blue());


        telemetry.addData("rightfront encoder", motorRightFront.getCurrentPosition());
        telemetry.addData("leftfront encoder", motorLeftFront.getCurrentPosition());
        telemetry.addData("leftback encoder", motorLeftBack.getCurrentPosition());
        telemetry.addData("rightback encoder", motorRightBack.getCurrentPosition());

        //telemetry.addData("gyro val", gyrosensor.getHeading());

        telemetry.addData("BLUE raw ultrasonic", BLUEsensorRange.rawUltrasonic());
        telemetry.addData("BLUE raw optical", BLUEsensorRange.rawOptical());
        telemetry.addData("BLUE cm optical", "%.2f cm", BLUEsensorRange.cmOptical());
        telemetry.addData("BLUE cm ultrasonic", "%.2f cm", BLUEsensorRange.cmUltrasonic());

        telemetry.addData("BLUE raw ultrasonic", BLUEsensorRange.rawUltrasonic());
        telemetry.addData("BLUE raw optical", BLUEsensorRange.rawOptical());
        telemetry.addData("BLUE cm optical", "%.2f cm", BLUEsensorRange.cmOptical());
        telemetry.addData("BLUE cm ultrasonic", "%.2f cm", BLUEsensorRange.cmUltrasonic());
        telemetry.addData("BLUE RANGE (full)  VALUE", "%.2f cm", BLUEsensorRange.getDistance(DistanceUnit.CM));

 */
        telemetry.addData("RED RANGE (full)  VALUE", "%.2f in", REDsensorRange.getDistance(DistanceUnit.INCH));
        telemetry.addData("BLUE RANGE (full)  VALUE", "%.2f in", BLUEsensorRange.getDistance(DistanceUnit.INCH));


        telemetry.addData("BLUE BACK - RED Value: ", BLUEbackColor.red());
        telemetry.addData("BLUE BACK - BLUE Value: ", BLUEbackColor.blue());

        telemetry.addData("BLUE FRONT - RED Value: ", BLUEfrontColor.red());
        telemetry.addData("BLUE FRONT - BLUE Value: ", BLUEfrontColor.blue());

        telemetry.addData("RED BACK - RED Value: ", REDbackColor.red());
        telemetry.addData("RED BACK - BLUE Value: ", REDbackColor.blue());

        telemetry.addData("RED FRONT - RED Value: ", REDfrontColor.red());
        telemetry.addData("RED FRONT - BLUE Value: ", REDfrontColor.blue());


 /*
        telemetry.addData("bottom color", bottomcolor.argb());
        telemetry.addData("front color", frontcolor.argb());
        telemetry.addData("back color", backcolor.argb());

        telemetry.addData("front alpha", frontcolor.alpha());
        telemetry.addData("back alpha", frontcolor.alpha());

        telemetry.addData("status", dof.getSystemStatus().toShortString());

        telemetry.addData("acceleration", dof.getAcceleration());
        telemetry.addData("linear acceleration", dof.getLinearAcceleration());

        telemetry.addData("linear velocity", dof.getVelocity());

        telemetry.addData("SERVO VALUE", blueServo.getPosition());

 */

        Orientation current_orientation = dof.getAngularOrientation();


        telemetry.addData("navx", navx_device.getYaw());

        telemetry.addData("Current orientation", -(current_orientation.firstAngle - angle_0));
        telemetry.update();

    }

    public void setMotorPower(){

    }

    public void setIntakePower(){

        if(gamepad1.right_trigger > gamepad1.left_trigger) {
            intake.setPower(Math.abs(gamepad1.right_trigger));
        } else if (gamepad1.left_trigger > gamepad1.right_trigger){
            intake.setPower(-(gamepad1.left_trigger));
        } else{
            intake.setPower(0);
        }

    }

    public void powerShooter(){

        if(gamepad1.a){
            shooter.setPower(-0.5);
        }else{
            shooter.setPower(0);
        }

    }



}
