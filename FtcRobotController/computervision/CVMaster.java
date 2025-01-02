package org.firstinspires.ftc.teamcode.computervision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.computervision.pipelines.ConeObserverPipeline;
import org.firstinspires.ftc.teamcode.computervision.pipelines.PixelObserverPipeline;
import org.firstinspires.ftc.teamcode.computervision.pipelines.PoleObserverPipeline;
import org.firstinspires.ftc.teamcode.computervision.pipelines.TSGObserverPipeline;
import org.firstinspires.ftc.teamcode.computervision.pipelines.teamElementRedPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class CVMaster {
    OpenCvWebcam webcam;
    public ConeObserverPipeline conePipeline;

    public PoleObserverPipeline polePipeline;
    public TSGObserverPipeline TSGPipeline;
    public PixelObserverPipeline PixelPipeline;
    public teamElementRedPipeline tsgRedPipeline;

    private LinearOpMode op;
    public CVMaster(LinearOpMode p_op){
        //you can input  a hardwareMap instead of linearOpMode if you want
        op = p_op;

        //initialize webcam
        int cameraMonitorViewId = op.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", op.hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(op.hardwareMap.get(WebcamName.class, "MantisCam"), cameraMonitorViewId);
    }
    public void observeTSGRed(){
        //create the pipeline
        tsgRedPipeline = new teamElementRedPipeline();
        webcam.setPipeline(tsgRedPipeline);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 10);
            }

            @Override
            public void onError(int errorCode) {}
        });

    }
    public void observeCone(){
        //create the pipeline
        conePipeline = new ConeObserverPipeline();
        webcam.setPipeline(conePipeline);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 10);
            }

            @Override
            public void onError(int errorCode) {}
        });

    }
    public void observeTSG(){
        //create the pipeline
        TSGPipeline = new TSGObserverPipeline();
        webcam.setPipeline(TSGPipeline);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 10);
            }

            @Override
            public void onError(int errorCode) {}
        });

    }
    public void observePixel(){
        //create the pipeline
        PixelPipeline = new PixelObserverPipeline();
        webcam.setPipeline(PixelPipeline);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 10);
            }

            @Override
            public void onError(int errorCode) {}
        });

    }
    public void observePole(){
        //create the pipeline
        polePipeline = new PoleObserverPipeline();
        webcam.setPipeline(polePipeline);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 10);
            }

            @Override
            public void onError(int errorCode) {}
        });

    }



    //stop streaming
    public void stopCamera(){
        webcam.stopStreaming();
    }


}