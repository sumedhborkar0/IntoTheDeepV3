package org.firstinspires.ftc.teamcode.computervision.pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class pixelDetectionPipeline extends OpenCvPipeline {
    Mat cbMat = new Mat();

    /*
     * Threshold values
     */
    Scalar lowerYellow = new Scalar(18,50, 70);
    Scalar upperYellow = new Scalar(29, 255, 255);
    Scalar lowerGreen = new Scalar(40, 52, 72);
    Scalar upperGreen = new Scalar(70, 255, 255);
    Scalar lowerWhite = new Scalar (0, 0, 215);
    Scalar upperWhite = new Scalar (255, 40, 255);
    Scalar lowerPurple = new Scalar (85, 50, 160);
    Scalar upperPurple = new Scalar (150, 190, 255);


  

    @Override
    public Mat processFrame(Mat input) {
        // We'll be updating this with new data below

        /*
         * Run the image processing
         */
        ArrayList<MatOfPoint> contourlist = new ArrayList<>();
        Imgproc.cvtColor(input, cbMat, Imgproc.COLOR_RGB2HSV);

        // Find Green mask
        Mat GreenMat = colorMask(cbMat, lowerGreen, upperGreen);
        // Find Yellow mask
        Mat YellowMat = colorMask(cbMat, lowerYellow, upperYellow);
        // Find White mask
        Mat WhiteMat = colorMask(cbMat, lowerWhite, upperWhite);
        // Find Purple mask
        Mat PurpleMat = colorMask(cbMat, lowerPurple, upperPurple);

        //This draws the contours for all of the colors

        Imgproc.findContours(GreenMat, contourlist, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        System.out.println("Green");
        for (int i = 0; i < contourlist.size(); i++) {
            Imgproc.drawContours(input, contourlist, i, new Scalar(23, 230, 180), 2);
            MatOfPoint2f mp1 = new MatOfPoint2f(contourlist.get(i).toArray());
            RotatedRect r1 = Imgproc.minAreaRect(mp1);
            Imgproc.putText(input,
                        "Green",
                            new Point(r1.center.x, r1.center.y),
                            Imgproc.FONT_HERSHEY_PLAIN,
                            1,
                            new Scalar(23, 230, 180),
                            1);

        }
        contourlist.clear();

        Imgproc.findContours(YellowMat, contourlist, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        System.out.println("Yellow");
        for (int x = 0; x < contourlist.size(); x++) {
            Imgproc.drawContours(input, contourlist, x, new Scalar (23, 230, 180), 2);
            MatOfPoint2f mp1 = new MatOfPoint2f(contourlist.get(x).toArray());
            RotatedRect r1 = Imgproc.minAreaRect(mp1);
            Imgproc.putText(input,
                    "Yellow",
                    new Point(r1.center.x, r1.center.y),
                    Imgproc.FONT_HERSHEY_PLAIN,
                    1,
                    new Scalar(23, 230, 180),
                    1);
        }
        contourlist.clear();

        Imgproc.findContours(WhiteMat, contourlist, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        System.out.println("White");
        for (int z = 0; z < contourlist.size(); z++) {
            Imgproc.drawContours(input, contourlist, z, new Scalar (23, 230, 180), 2);
            MatOfPoint2f mp1 = new MatOfPoint2f(contourlist.get(z).toArray());
            RotatedRect r1 = Imgproc.minAreaRect(mp1);
            Imgproc.putText(input,
                    "White",
                    new Point(r1.center.x, r1.center.y),
                    Imgproc.FONT_HERSHEY_PLAIN,
                    1,
                    new Scalar(23, 230, 180),
                    1);
        }

        contourlist.clear();

        Imgproc.findContours(PurpleMat, contourlist, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        System.out.println("Purple");
        for (int y = 0; y < contourlist.size(); y++) {
            Imgproc.drawContours(input, contourlist, y, new Scalar (23, 230, 180), 2);
            MatOfPoint2f mp1 = new MatOfPoint2f(contourlist.get(y).toArray());
            RotatedRect r1 = Imgproc.minAreaRect(mp1);
            Imgproc.putText(input,
                    "Purple",
                    new Point(r1.center.x, r1.center.y),
                    Imgproc.FONT_HERSHEY_PLAIN,
                    1,
                    new Scalar(23, 230, 180),
                    1);
        }
        return  input;
    }

    public Mat colorMask(Mat input, Scalar lowerVal, Scalar upperVal) {
        Mat outputMask = new Mat();
        Core.inRange(cbMat, lowerVal, upperVal, outputMask);
        return outputMask;
    }


}
