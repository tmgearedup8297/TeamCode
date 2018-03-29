package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@Autonomous(name="CVJewelVuforiaTestBlue", group="Linear Opmode")
//@Disabled
public class CVJewelVuforiaTestBlue extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private int pos=1;

    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;


    //private Servo glyphLeftDown = null;


    private Servo shoulderleft = null;



    static final double LEFT_SHOULDER_IN = 0.12;
    static final double LEFT_SHOULDER_OUT = 0.57;

   

    private JewelDetector jewelDetector = null;


    @Override
    public void runOpMode() {
        shoulderleft = hardwareMap.servo.get("shoulderLeft");
       


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

        String jewelOrder = jewelDetector.getCurrentOrder().toString();
        telemetry.addData("READING","");
        telemetry.update();
        sleep(3000);

        if (jewelOrder.equals("BLUE_RED")) {
            shoulderleft.setPosition(LEFT_SHOULDER_OUT + .2);
            sleep(1000);
            shoulderleft.setPosition(LEFT_SHOULDER_IN);
            sleep(1000);
        } else {
            shoulderleft.setPosition(LEFT_SHOULDER_OUT - .2);
            sleep(1000);
            shoulderleft.setPosition(LEFT_SHOULDER_IN);
            sleep(1000);
        }

            jewelDetector.disable();

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            vuforiaParameters.vuforiaLicenseKey = "AVUcFCn/////AAAAGRAznpbgTUrbnvSn5Odx3WFSWKyWk+CaQKGEJCcm033tEoqUiLMTlyHFwX01tkB5QfFVbnJAp8kTS442QovsVniqOrTAxcbKJHNzbYEVtYx/4ZDyIS7Vsb+uyE2VSNs8pKEPmsZAmW9/XJid02yP7/2K6W1nJ6NpwFKdY3qLs/7qX+M4CPeCkKdnllkgjsq99xvIOncxGorzdjNIAYfIEwHds0BhKKcR3GyyflHAOx0zvTAEvu8g0g2kSTDPxemDY5vug2O0vE61mW/AL5YIIlJeWf3dfpPJg7SjP8RxS44vMoW8bpEkfJXD2KhZpigDGP+SV1JHLz2d3DrpP+TBFSQG00+F93Dd7gofT8n7rpyk";
            vuforiaParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
            this.vuforia = ClassFactory.createVuforiaLocalizer(vuforiaParameters);
            VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
            VuforiaTrackable relicTemplate = relicTrackables.get(0);
            relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
            relicTrackables.activate();

        telemetry.addData("READING","");
        telemetry.update();
        sleep(3000);

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        String poString = vuMark.toString();

        if(poString.equals("LEFT")){
            pos=0;
        }else if(poString.equals("CENTER")){
            pos=1;
        }else if(poString.equals("RIGHT")) {
            pos=2;
        }


        telemetry.addData("Pos", pos);
        telemetry.addData("VuMark", vuMark);
        telemetry.update();

        if(pos==0)
            shoulderleft.setPosition(LEFT_SHOULDER_IN+.1);
        else if(pos==1)
            shoulderleft.setPosition(LEFT_SHOULDER_OUT);
        else
            shoulderleft.setPosition(LEFT_SHOULDER_OUT+.1);
    }


            }


//}
