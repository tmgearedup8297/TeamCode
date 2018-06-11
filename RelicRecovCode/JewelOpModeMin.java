
package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;


@Autonomous(name="DogeCV Jewel Detector MIN", group="DogeCV")
//@Disabled
public class JewelOpModeMin extends LinearOpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    private JewelDetector jewelDetector = null;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");


        jewelDetector = new JewelDetector();
        jewelDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        //Jewel Detector Settings
        jewelDetector.areaWeight = 0.02;
        jewelDetector.detectionMode = JewelDetector.JewelDetectionMode.MAX_AREA; // PERFECT_AREA
        //jewelDetector.perfectArea = 6500; <- Needed for PERFECT_AREA
        jewelDetector.debugContours = true;
        jewelDetector.maxDiffrence = 500;
        jewelDetector.ratioWeight = 15;
        jewelDetector.perfectArea = 4000;
        jewelDetector.perfectRatio = 1;
        jewelDetector.areaWeight = .05;
        jewelDetector.minArea = 5000;
        jewelDetector.downScaleFactor = 0.4;
        jewelDetector.enable();

        waitForStart();
        Map<String, Integer> map = new HashMap<String,Integer>();
        double start = runtime.seconds();
        while(runtime.seconds()<start+3){
            String s = jewelDetector.getCurrentOrder().toString();
            if(map.keySet().contains(s)){
                map.put(s,map.get(s)+1);
            }
            else if(s.length()>=8){
                map.put(s,1);
            }

        }
        int max = -1;
        String maxval = "";
        for(String s:map.keySet()){
            if(map.get(s)>max){
                max = map.get(s);
                maxval = s;
            }
        }
        telemetry.addData("Jewel Order: ", maxval);
        telemetry.update();

        sleep(30000);
        jewelDetector.disable();
    }


}
