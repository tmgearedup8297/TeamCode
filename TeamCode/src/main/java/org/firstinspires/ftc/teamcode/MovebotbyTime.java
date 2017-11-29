import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 770742 on 11/26/2017.
 */
@Autonomous(name = "MovebotbyTime", group = "Autonomous")
public class MovebotbyTime extends MasterDeviceClass {
    static final double power=0.25;
    static final int time=1500;
    @Override
    public void runOpMode() throws InterruptedException {
    initializeHardware();
        waitForStart();
        motorLeftBack.setPower(power);
        motorRightFront.setPower(power);
        motorLeftFront.setPower(power);
        motorRightFront.setPower(power);
        sleep(time);
        motorLeftBack.setPower(0);
        motorRightFront.setPower(0);
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
        exitOpMode();
    }
}
//You'll have to adjust the power variable to account for all 4 wheels
//Extends the Master Device class from last year