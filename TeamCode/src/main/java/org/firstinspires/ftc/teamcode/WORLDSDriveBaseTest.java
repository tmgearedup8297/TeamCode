package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


@Autonomous(name="WORLDSDriveBaseTest", group="Linear Opmode")
//@Disabled
public class WORLDSDriveBaseTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private final int TICKS_PER_INCH=36;


    private int[] zeroPos = {0, 0, 0, 0};
    private double[] targetDist = {24, 24, 24, 24};
    private int[] targetClicks = {0, 0, 0, 0};
    private int pos=1;

    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;


    //private Servo glyphLeftDown = null;


    private Servo shoulderleft = null;
    private Servo elbowLeft = null;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor lift = null;
    private Servo autoGlyphLeft = null;



    static final double LEFT_SHOULDER_IN = 0.12;
    static final double LEFT_SHOULDER_OUT = 0.57;
    static final double LEFT_SHOULDER_OUT_JEWEL = 0.55;
    //static final double RIGHT_SHOULDER_IN = 0.175;
    //static final double RIGHT_SHOULDER_OUT = 0.732;
    static final double LEFT_ELBOW_OUT = .31;
    static final double LEFT_ELBOW_MID = 0.62;
    static final double LEFT_ELBOW_IN = 0.84;
    //static final double RIGHT_ELBOW_OUT = 0.26;
    //static final double RIGHT_ELBOW_IN = 0.94;
    static final double LEFT_AUTOGLYPH_IN = 0.0;
    static final double LEFT_AUTOGLYPH_OUT = 1.0;
    static final double RIGHT_AUTOGLYPH_IN = 0.0;
    static final double RIGHT_AUTOGLYPH_OUT = 1.0;
    static final double ACTIVATOR_IN = 0.0;
    static final double ACTIVATOR_OUT = 0.65;


    private DigitalChannel leftLim;

    private JewelDetector jewelDetector = null;


    @Override
    public void runOpMode() {
        shoulderleft = hardwareMap.servo.get("shoulderleft");
        elbowLeft = hardwareMap.servo.get("elbowLeft");
        leftLim = hardwareMap.get(DigitalChannel.class, "leftLim");

        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        lift = hardwareMap.get(DcMotor.class, "lift");
        autoGlyphLeft = hardwareMap.get(Servo.class, "autoGlyphLeft");
        autoGlyphLeft.setDirection(Servo.Direction.REVERSE);
        jewelDetector = new JewelDetector();
        jewelDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        //Jewel Detector Settings
        jewelDetector.areaWeight = 0.02;
        jewelDetector.detectionMode = JewelDetector.JewelDetectionMode.MAX_AREA; // PERFECT_AREA
        //jewelDetector.perfectArea = 6500; <- Needed for PERFECT_AREA
        jewelDetector.debugContours = true;
        jewelDetector.maxDiffrence = 15;
        jewelDetector.ratioWeight = 15;
        jewelDetector.minArea = 700;
        jewelDetector.enable();

        waitForStart();
        runtime.reset();

        jewelDetector.disable();


        moveDistForward(50,50,50,50);
        brake();
        sleep(1000);


        telemetry.addData("moving backward","");
        telemetry.update();
        moveDistBack(20,20,20,20);
        brake();
        sleep(1000);



        telemetry.addData("strafing left","");
        telemetry.update();
        strafeDistLeft(20,20,20,20);
        brake();
        sleep(1000);


        telemetry.addData("strafing right","");
        telemetry.update();
        strafeDistRight(20,20,20,20);
        brake();
        sleep(1000);


        telemetry.addData("turning right","");
        telemetry.update();
        turnRight(90);
        brake();
        sleep(1000);


        telemetry.addData("turning left","");
        telemetry.update();

        turnLeft(90);
        brake();
        sleep(1000);


        straightForward(10,10,10,10);
        brake();
        sleep(1000);



    }

    public void straightForward(double leftFrontTargetDist, double leftBackTargetDist, double rightFrontTargetDist, double rightBackTargetDist) {

        targetDist[0] = leftFrontTargetDist;
        targetDist[1] = leftBackTargetDist;
        targetDist[2] = rightFrontTargetDist;
        targetDist[3] = rightBackTargetDist;

        zeroPos[0] = leftFront.getCurrentPosition();
        zeroPos[1] = leftBack.getCurrentPosition();
        zeroPos[2] = rightFront.getCurrentPosition();
        zeroPos[3] = rightBack.getCurrentPosition();

        targetClicks[0] = (int)(targetDist[0] * TICKS_PER_INCH);
        targetClicks[1] = (int)(targetDist[1] * TICKS_PER_INCH);;
        targetClicks[2] = (int)(targetDist[2] * TICKS_PER_INCH);;
        targetClicks[3]= (int)(targetDist[3] * TICKS_PER_INCH);;

        leftFront.setPower(-.1);
        leftBack.setPower(-.1);
        rightFront.setPower(-.115);
        rightBack.setPower(-.115);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        while(opModeIsActive() && leftFront.getCurrentPosition()<zeroPos[0]+targetClicks[0] &&
                leftBack.getCurrentPosition()<zeroPos[1]+targetClicks[1] &&
                rightFront.getCurrentPosition()<zeroPos[2]+targetClicks[2] &&
                rightBack.getCurrentPosition()<zeroPos[3]+targetClicks[3]){



        }
    }

    public void strafeDistRight(double leftFrontTargetDist, double leftBackTargetDist, double rightFrontTargetDist, double rightBackTargetDist) {


        targetDist[0] = leftFrontTargetDist;
        targetDist[1] = leftBackTargetDist;
        targetDist[2] = rightFrontTargetDist;
        targetDist[3] = rightBackTargetDist;

        zeroPos[0] = leftFront.getCurrentPosition();
        zeroPos[1] = leftBack.getCurrentPosition();
        zeroPos[2] = rightFront.getCurrentPosition();
        zeroPos[3] = rightBack.getCurrentPosition();

        targetClicks[0] = (int) (targetDist[0] * TICKS_PER_INCH);
        targetClicks[1] = (int) (targetDist[1] * TICKS_PER_INCH);
        targetClicks[2] = (int) (targetDist[2] * TICKS_PER_INCH);
        targetClicks[3] = (int) (targetDist[3] * TICKS_PER_INCH);

        //THIS IS GOOD CODDDDDDDDDeeee
        /*leftFront.setPower(.14);
        leftBack.setPower(-.1);
        rightFront.setPower(.1);
        rightBack.setPower(-.14);*/

        leftFront.setPower(.13);
        leftBack.setPower(-.1);
        rightFront.setPower(-.13);
        rightBack.setPower(.1);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        while (opModeIsActive() && leftFront.getCurrentPosition() < zeroPos[0] + targetClicks[0] &&
                rightBack.getCurrentPosition() < zeroPos[3] + targetClicks[3]) {

        }
        brake();
    }
    public void strafeDistLeft(double leftFrontTargetDist, double leftBackTargetDist, double rightFrontTargetDist, double rightBackTargetDist){


        targetDist[0] = leftFrontTargetDist;
        targetDist[1] = leftBackTargetDist;
        targetDist[2] = rightFrontTargetDist;
        targetDist[3] = rightBackTargetDist;

        zeroPos[0] = leftFront.getCurrentPosition();
        zeroPos[1] = leftBack.getCurrentPosition();
        zeroPos[2] = rightFront.getCurrentPosition();
        zeroPos[3] = rightBack.getCurrentPosition();

        targetClicks[0] = (int)(targetDist[0] * TICKS_PER_INCH);
        targetClicks[1] = (int)(targetDist[1] * TICKS_PER_INCH);;
        targetClicks[2] = (int)(targetDist[2] * TICKS_PER_INCH);;
        targetClicks[3]= (int)(targetDist[3] * TICKS_PER_INCH);;

        //THIS IS GOOD CODEEEEEEEEEEEEEEEE
        /*leftFront.setPower(-.14);
        leftBack.setPower(.1);
        rightFront.setPower(-.14);
        rightBack.setPower(.12);*/

        leftFront.setPower(-.1);
        leftBack.setPower(.1);
        rightFront.setPower(.1);
        rightBack.setPower(-.1);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() && leftFront.getCurrentPosition() > zeroPos[0] - targetClicks[0] &&
                rightBack.getCurrentPosition() > zeroPos[3] - targetClicks[3]) {

            telemetry.update();
        }


    }
    public void turnRight(double degrees){
        double dist = 12.5*Math.PI *(degrees/360);
        double leftFrontTargetDist = dist;
        double leftBackTargetDist = dist;
        double rightFrontTargetDist = dist;
        double rightBackTargetDist = dist;

        targetDist[0] = leftFrontTargetDist;
        targetDist[1] = leftBackTargetDist;
        targetDist[2] = rightFrontTargetDist;
        targetDist[3] = rightBackTargetDist;

        zeroPos[0] = leftFront.getCurrentPosition();
        zeroPos[1] = leftBack.getCurrentPosition();
        zeroPos[2] = rightFront.getCurrentPosition();
        zeroPos[3] = rightBack.getCurrentPosition();

        targetClicks[0] = (int)(targetDist[0] * TICKS_PER_INCH);
        targetClicks[1] = (int)((int)(targetDist[1] * TICKS_PER_INCH));
        targetClicks[2] = (int)((int)(targetDist[2] * TICKS_PER_INCH));
        targetClicks[3]= (int)((int)(targetDist[3] * TICKS_PER_INCH));

        leftFront.setPower(-.1);
        leftBack.setPower(-.1);
        rightFront.setPower(.1);
        rightBack.setPower(.1);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        while(opModeIsActive() && leftFront.getCurrentPosition()>zeroPos[0]-targetClicks[0] &&
                leftBack.getCurrentPosition()>zeroPos[1]-targetClicks[1] &&
                rightFront.getCurrentPosition()<zeroPos[2]+targetClicks[2] &&
                rightBack.getCurrentPosition()<zeroPos[3]+targetClicks[3]){


        }

    }
    public void turnLeft(double degrees){
        double dist = 12.5*Math.PI *(degrees/360);
        double leftFrontTargetDist = dist;
        double leftBackTargetDist = dist;
        double rightFrontTargetDist = dist;
        double rightBackTargetDist = dist;

        targetDist[0] = leftFrontTargetDist;
        targetDist[1] = leftBackTargetDist;
        targetDist[2] = rightFrontTargetDist;
        targetDist[3] = rightBackTargetDist;

        zeroPos[0] = leftFront.getCurrentPosition();
        zeroPos[1] = leftBack.getCurrentPosition();
        zeroPos[2] = rightFront.getCurrentPosition();
        zeroPos[3] = rightBack.getCurrentPosition();

        targetClicks[0] = (int)(targetDist[0] * TICKS_PER_INCH);
        targetClicks[1] = (int)((int)(targetDist[1] * TICKS_PER_INCH));
        targetClicks[2] = (int)((int)(targetDist[2] * TICKS_PER_INCH));
        targetClicks[3]= (int)((int)(targetDist[3] * TICKS_PER_INCH));

        leftFront.setPower(.1);
        leftBack.setPower(.1);
        rightFront.setPower(-.1);
        rightBack.setPower(-.1);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        while(opModeIsActive() && leftFront.getCurrentPosition()<zeroPos[0]+targetClicks[0]){

        }

    }
    public void moveDistBack(double leftFrontTargetDist, double leftBackTargetDist, double rightFrontTargetDist, double rightBackTargetDist){

        targetDist[0] = leftFrontTargetDist;
        targetDist[1] = leftBackTargetDist;
        targetDist[2] = rightFrontTargetDist;
        targetDist[3] = rightBackTargetDist;

        zeroPos[0] = leftFront.getCurrentPosition();
        zeroPos[1] = leftBack.getCurrentPosition();
        zeroPos[2] = rightFront.getCurrentPosition();
        zeroPos[3] = rightBack.getCurrentPosition();

        targetClicks[0] = (int)(targetDist[0] * TICKS_PER_INCH);
        targetClicks[1] = (int)(targetDist[1] * TICKS_PER_INCH);;
        targetClicks[2] = (int)(targetDist[2] * TICKS_PER_INCH);;
        targetClicks[3]= (int)(targetDist[3] * TICKS_PER_INCH);;

        leftFront.setPower(-.1);
        leftBack.setPower(-.1);
        rightFront.setPower(-.115);
        rightBack.setPower(-.115);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        while(opModeIsActive() && leftFront.getCurrentPosition()>zeroPos[0]-targetClicks[0] &&
                leftBack.getCurrentPosition()>zeroPos[1]-targetClicks[1] &&
                rightFront.getCurrentPosition()>zeroPos[2]-targetClicks[2] &&
                rightBack.getCurrentPosition()>zeroPos[3]-targetClicks[3]){


        }

    }
    public void moveDistForward(double leftFrontTargetDist, double leftBackTargetDist, double rightFrontTargetDist, double rightBackTargetDist){

        targetDist[0] = leftFrontTargetDist;
        targetDist[1] = leftBackTargetDist;
        targetDist[2] = rightFrontTargetDist;
        targetDist[3] = rightBackTargetDist;

        zeroPos[0] = leftFront.getCurrentPosition();
        zeroPos[1] = leftBack.getCurrentPosition();
        zeroPos[2] = rightFront.getCurrentPosition();
        zeroPos[3] = rightBack.getCurrentPosition();

        targetClicks[0] = (int)(targetDist[0] * TICKS_PER_INCH);
        targetClicks[1] = (int)(targetDist[1] * TICKS_PER_INCH);;
        targetClicks[2] = (int)(targetDist[2] * TICKS_PER_INCH);;
        targetClicks[3]= (int)(targetDist[3] * TICKS_PER_INCH);;

        leftFront.setPower(.1);
        leftBack.setPower(.1);
        rightFront.setPower(.115);
        rightBack.setPower(.115);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        while(opModeIsActive() && leftFront.getCurrentPosition()<zeroPos[0]+targetClicks[0] &&
                leftBack.getCurrentPosition()<zeroPos[1]+targetClicks[1] &&
                rightFront.getCurrentPosition()<zeroPos[2]+targetClicks[2] &&
                rightBack.getCurrentPosition()<zeroPos[3]+targetClicks[3]){

                telemetry.addData("Moving forward","");
                telemetry.addData("Left Front pos", leftFront.getCurrentPosition());
                telemetry.addData("LeftFront target", zeroPos[0]+targetClicks[0]);

            telemetry.addData("Right Front pos", rightFront.getCurrentPosition());
            telemetry.addData("RightFront target", zeroPos[2]+targetClicks[2]);

            telemetry.addData("Left Back pos", leftFront.getCurrentPosition());
            telemetry.addData("LeftBack target", zeroPos[1]+targetClicks[1]);

            telemetry.addData("Right Back pos", leftFront.getCurrentPosition());
            telemetry.addData("RightBack target", zeroPos[3]+targetClicks[3]);

            telemetry.update();
        }

    }
    public void brake() {
        leftFront.setPower(0.01);
        leftBack.setPower(-0.01);
        rightFront.setPower(0.01);
        rightBack.setPower(-0.01);
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}



//}
