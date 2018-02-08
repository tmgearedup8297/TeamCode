package com.disnodeteam.dogecv.detectors;

import com.disnodeteam.dogecv.OpenCVPipeline;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class CryptoboxDetector extends OpenCVPipeline {

    public double downScaleFactor = 0.6;

    private Scalar lower = new Scalar(90, 135, 25);
    private Scalar upper = new Scalar(130, 250, 150);

    private Mat workingMat = new Mat();
    private Mat mask = new Mat();
    private Mat hsv = new Mat();
    private Mat structure = new Mat();
    private Mat hierarchy = new Mat();
    private Mat kernel = Mat.ones(5, 5, CvType.CV_32F);


    private double distanceToMove = 100;

    @Override
    public Mat processFrame(Mat rgba, Mat gray) {

        Size initSize= rgba.size();
        Size newSize  = new Size(initSize.width * downScaleFactor, initSize.height * downScaleFactor);
        rgba.copyTo(workingMat);

        Imgproc.resize(workingMat, workingMat, newSize);

        Mat tempBefore = workingMat.t();

        Core.transpose(tempBefore, workingMat);
        Core.flip(workingMat, workingMat, 1);
        Core.flip(workingMat, workingMat, 0);

        tempBefore.release();

        List<MatOfPoint> contours = new ArrayList<>();
        List<Rect> columns = new ArrayList<>();

        Imgproc.erode(workingMat, workingMat,kernel);
        Imgproc.dilate(workingMat, workingMat,kernel);
        Imgproc.cvtColor(workingMat, hsv, Imgproc.COLOR_RGB2HSV);


        Core.inRange(hsv, lower, upper, mask);

        Imgproc.blur(hsv, hsv, new Size(5,5));

        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        List<Double> contourArea = new ArrayList<>();

        for(MatOfPoint c : contours) {
            contourArea.add(Imgproc.contourArea(c));
        }

        if (contourArea.size() >= 2) {
            double maxArea = Collections.max(contourArea);
            int indexOfMax = contourArea.indexOf(maxArea);

            columns.add(Imgproc.boundingRect(contours.get(indexOfMax)));

            contours.remove(indexOfMax);
            contourArea.remove(indexOfMax);

            maxArea = Collections.max(contourArea);
            indexOfMax = contourArea.indexOf(maxArea);

            columns.add(Imgproc.boundingRect(contours.get(indexOfMax)));

            for (Rect box : columns) {
                Imgproc.rectangle(workingMat, new Point(box.x, box.y), new Point(box.x + box.width, box.y + box.height),
                        new Scalar(255, 0, 0), 2);
            }

            Collections.sort(columns, new Comparator<Rect>() {
                @Override
                public int compare(Rect rect, Rect t1) {
                    if (rect.y > t1.y) {
                        return 1;
                    } else if (rect.y < t1.y) {
                        return -1;
                    } else {
                        return 0;
                    }
                }
            });

            Rect leftColumn = columns.get(0);
            Rect rightColumn = columns.get(1);

            double leftMiddle = leftColumn.y + (leftColumn.height / 2);
            double rightMiddle = rightColumn.y + (rightColumn.height / 2);

            double pixelDivisor = (rightMiddle - leftMiddle) / 7.63;

            double middleOfColumn = (rightMiddle + leftMiddle) / 2;

            Imgproc.line(workingMat, new Point(0, middleOfColumn),
                    new Point(workingMat.size().width, middleOfColumn), new Scalar(0, 0, 255), 1);

            Imgproc.circle(workingMat, new Point(workingMat.size().width / 2, middleOfColumn), 4,
                    new Scalar(0, 0, 255));

            distanceToMove = (middleOfColumn - (workingMat.size().height / 2)) / pixelDivisor;
        }

        Imgproc.line(workingMat, new Point(0, workingMat.size().height / 2),
                new Point(workingMat.size().width, workingMat.size().height / 2), new Scalar(0, 255, 0), 1);


        Mat tempAfter = workingMat.t();

        Core.flip(tempAfter, workingMat, 0);

        tempAfter.release();

        Imgproc.resize(workingMat, workingMat, initSize);

        return workingMat;
    }

    public void setColor(String color) {
        if (color.equals("Blue")) {
            lower = new Scalar(80, 70, 0);
            upper = new Scalar(160, 255, 130);
        } else if (color.equals("Red")) {
            lower = new Scalar(30, 100, 110);
            upper = new Scalar(255, 255, 130);
        }
    }

    public double getDistanceToMove() {
        return distanceToMove;
    }
}
