package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name = "BLUEShooter-TWO-Beacons-SPIN TURN", group = "BLUEShooter-TWO-Beacons-SPIN TURN")
@Disabled

public class BLUEShooterTWOBeaconsSpinTurn extends ShooterTWOBeaconsSpinTurn {

    @Override
    public void runOpMode() throws InterruptedException {

        initializeHardware();

        setAttributes(2240, 4, (float) 12);

        waitForStart();

        activateShooterTWOBeaconsSpinTurn("blue"); // since we are NOT using a generic version of this method for WVA tournament

        exitOpMode();

    }

}
