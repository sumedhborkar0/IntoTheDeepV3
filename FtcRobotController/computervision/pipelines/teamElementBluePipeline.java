package org.firstinspires.ftc.teamcode.computervision.pipelines;

import static org.opencv.imgproc.Imgproc.rectangle;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class teamElementBluePipeline extends OpenCvPipeline {

//use 176 by 144 pixel camera
    public static int x1 =0,x2=50,x3=60,x4=110,x5=120,x6=170,y1=10,y2=125;
    public static Mat stream = new Mat();
    public static int maxAt = 2;

    //methods to configure bounding boxes

    public static void configureRects(int z1,int z2,int z3, int z4, int z5, int z6, int p1, int p2){
        x1 = z1;
        x2 = z2;
        x3 = z3;
        x4 = z4;
        x5 = z5;
        x6 = z6;
        y1 = p1;
        y2 = p2;
    }
    public static void translateRects(int leftX, int middleX, int rightX, int y){
        x1 += leftX;
        x2 += leftX;

        x3 += middleX;
        x4 += middleX;

        x5 += rightX;
        x6 += rightX;

        y1 += y;
        y2 += y;


    }
    @Override
    public Mat processFrame(Mat input){

        stream = input;






        int avg_blueLeft = 0;
        int avg_blueMiddle = 0;
        int avg_blueRight = 0;
        // Blue value is red 0, blue 255 green 0
        double red_threshold = 150;
        double green_threshold = 50;
        double blue_threshold = 50;

        //goes through each pixel in bounding box one
        for(int x = x1; x<=x2; x++) {
            for(int y = y1; y<=y2; y++) {
                double[] pixel1 = stream.get(y, x);
                if (pixel1 == null) {
                    continue;
                }
                double b1 = pixel1[0] ;
                double g1 = pixel1[1] ;
                double r1 = pixel1[2] ;

                System.out.println("r1: " + r1 + " b1: " + b1 + " g1: " +  g1);
                if (r1 >= red_threshold && g1 >= green_threshold && b1 <= blue_threshold) {
                    avg_blueLeft += 1;
                }

            }
        }
        System.out.print("Left:");
        System.out.println(avg_blueLeft);
        //process repeated for each bounding box
        for(int x = x3; x<=x4; x++) {
            for(int y = y1; y<=y2; y++) {
                double[] pixel1 = stream.get(y,x);
                if (pixel1 == null) {
                    continue;
                }
                double b1 = pixel1[0] ;
                double g1 = pixel1[1] ;
                double r1 = pixel1[2] ;

                //System.out.println("r1: " + r1 + " b1: " + b1 + " g1: " +  g1);
                if (r1 >= red_threshold && g1 >= green_threshold && b1 <= blue_threshold) {
                    avg_blueMiddle += 1;
                }
            }
        }
        System.out.print("Middle:");
        System.out.println(avg_blueMiddle);

        for(int x = x5; x<=x6; x++) {
            for(int y = y1; y<=y2; y++) {
                double[] pixel1 = stream.get(y,x);
                if (pixel1 == null) {
                    continue;
                }
                double b1 = pixel1[0] ;
                double g1 = pixel1[1] ;
                double r1 = pixel1[2] ;

                //System.out.println("r1: " + r1 + " b1: " + b1 + " g1: " +  g1);
                if (r1 >= red_threshold && g1 >= green_threshold && b1 <= blue_threshold) {
                    avg_blueRight += 1;
                }
            }
        }
        System.out.print("Right:");
        System.out.println(avg_blueRight);


        //code to compare which bounding box has the highest number of green pixels and draw that on screen
        int[] valuesArray = {0,0,0};
        valuesArray[0] = avg_blueLeft;

        valuesArray[1] = avg_blueMiddle;

        valuesArray[2] = avg_blueRight;



        for (int i = 0; i < valuesArray.length; i++) {
            maxAt = valuesArray[i] > valuesArray[maxAt] ? i : maxAt;
        }

        if(maxAt == 0){
            Imgproc.rectangle(input,new Point(x1 , y1),new Point(x2,y2),new Scalar(0,0,255),1);

            Imgproc.rectangle(input,new Point(x3 , y1),new Point(x4,y2),new Scalar(0,0,0),1);

            Imgproc.rectangle(input,new Point(x5 , y1),new Point(x6,y2),new Scalar(0,0,0),1);

            //telemetry.addData("Left", maxAt);
            //telemetry.update();
        }
        if(maxAt == 1){
            Imgproc.rectangle(input,new Point(x1 , y1),new Point(x2,y2),new Scalar(0,0,0),1);

            Imgproc.rectangle(input,new Point(x3 , y1),new Point(x4,y2),new Scalar(0,0,255),1);

            Imgproc.rectangle(input,new Point(x5 , y1),new Point(x6,y2),new Scalar(0,0,0),1);

            //telemetry.addData("Middle", maxAt);
            //telemetry.update();
        }
        if(maxAt == 2){
            Imgproc.rectangle(input,new Point(x1 , y1),new Point(x2,y2),new Scalar(0,0,0),1);

            Imgproc.rectangle(input,new Point(x3 , y1),new Point(x4,y2),new Scalar(0,0,0),1);

            Imgproc.rectangle(input,new Point(x5 , y1),new Point(x6,y2),new Scalar(0,0,255),1);

            //telemetry.addData("Right", maxAt);
            //telemetry.update();
        }
        return input;
    }



    //returns the location of the TSE

    public static int findTSE(){



        return maxAt;


    }
}
