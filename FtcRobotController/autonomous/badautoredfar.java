package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.computervision.CVMaster;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "badautoredfar", preselectTeleOp = "teleop")
public class badautoredfar extends LinearOpMode {
    public DcMotor intake;
    public Servo servoleft;
    public Servo servoright;
    public DcMotor slidesleft;
    public DcMotor slidesright;
    public Servo micro;
    public String parkposition = "right";
    public boolean delayed = false;
    public int location;
    private PIDController slidescontroller;
    public static double ps = 0.002, is = 0, ds = 0;
    public static double fs = 0.01;
    public int targets = 0;
    private final double ticks_in_degrees = 384 / 360.0;
    public double rightX = 55;
    public double rightY = -45;
    public double midX = 54.5;
    public double midY = -38.5;
    public double leftX = 55;
    public double leftY = -33.75;

    public int rightHeight = 1600;
    public int midHeight = 1600;
    public int leftHeight = 1600;


    public void runOpMode(){
        CVMaster cv = new CVMaster(this);
        cv.observeTSG();

        telemetry.addData("tsg location", cv.TSGPipeline.coneCenterX);
        telemetry.addData("size", cv.TSGPipeline.distance);
        telemetry.update();
        slidescontroller = new PIDController(ps, is, ds);
        slidesleft = hardwareMap.dcMotor.get("slidesleft");
        slidesleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        micro = hardwareMap.servo.get("micro");
        servoleft = hardwareMap.servo.get("servoleft");
        servoright = hardwareMap.servo.get("servoright");
        servoleft.setPosition(0.2);
        servoright.setPosition(0.8);
        micro.setPosition(1);
        intake = hardwareMap.dcMotor.get("intake");

        slidesleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//slidesleft.setDirection(DcMotor.Direction.REVERSE);


        slidesright = hardwareMap.dcMotor.get("s");
        slidesright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesright.setDirection(DcMotor.Direction.REVERSE);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        drive.setPoseEstimate(new Pose2d(-40.86, -64.45, Math.toRadians(270.00)));
        TrajectorySequence closebardright = drive.trajectorySequenceBuilder(new Pose2d(-40.86, -64.45, Math.toRadians(270.00)))
                .setReversed(true)
                .splineTo(new Vector2d(-37.88, -32.66), Math.toRadians(90.00))
                .splineTo(new Vector2d(-29 , -34.73), Math.toRadians(0.00))
                .lineTo(new Vector2d(-52.37, -30.03))
                .lineTo(new Vector2d(-52.37, -59.18))
                .lineTo(new Vector2d(38.28, -60.41))
                .addTemporalMarker(()->{
                    targets = rightHeight;
                    servoleft.setPosition(0.5);
                    servoright.setPosition(0.5);
                })
                .splineTo(new Vector2d(rightX, rightY), Math.toRadians(0.00))
                .addTemporalMarker(() ->{
                    micro.setPosition(0.5);
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(46,-44.67))
                .lineTo(new Vector2d(35.82, -60.94))
                .addTemporalMarker(() -> {
                    targets = 0;
                    servoleft.setPosition(0.02);
                    servoright.setPosition(0.98);
                    micro.setPosition(1);
                })
                .lineTo(new Vector2d(60.59, -61.11))
                .build();
        TrajectorySequence closebardrightdelayed = drive.trajectorySequenceBuilder(new Pose2d(-40.86, -64.45, Math.toRadians(270.00)))
                .waitSeconds(7)
                .setReversed(true)
                .splineTo(new Vector2d(-37.88, -32.66), Math.toRadians(90.00))
                .splineTo(new Vector2d(-29 , -34.73), Math.toRadians(0.00))
                .lineTo(new Vector2d(-52.37, -30.03))
                .lineTo(new Vector2d(-52.37, -59.18))
                .lineTo(new Vector2d(38.28, -60.41))
                .addTemporalMarker(()->{
                    targets = rightHeight;
                    servoleft.setPosition(0.5);
                    servoright.setPosition(0.5);
                })
                .splineTo(new Vector2d(rightX, rightY), Math.toRadians(0.00))
                .addTemporalMarker(() ->{
                    micro.setPosition(0.5);
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(46,-44.67))
                .lineTo(new Vector2d(35.82, -60.94))
                .addTemporalMarker(() -> {
                    targets = 0;
                    servoleft.setPosition(0.02);
                    servoright.setPosition(0.98);
                    micro.setPosition(1);
                })
                .lineTo(new Vector2d(60.59, -61.11))
                .build();
        TrajectorySequence closebardleft = drive.trajectorySequenceBuilder(new Pose2d(-40.86, -64.45, Math.toRadians(270.00)))
                .setReversed(true)
                .splineTo(new Vector2d(-37.88, -32.66), Math.toRadians(90.00))
                .splineTo(new Vector2d(-29 , -34.73), Math.toRadians(0.00))
                .lineTo(new Vector2d(-52.37, -30.03))
                .lineTo(new Vector2d(-52.37, -59.18))
                .lineTo(new Vector2d(38.28, -60.41))
                .addTemporalMarker(()->{
                    targets = rightHeight;
                    servoleft.setPosition(0.5);
                    servoright.setPosition(0.5);
                })
                .splineTo(new Vector2d(rightX, rightY), Math.toRadians(0.00))
                .addTemporalMarker(() ->{
                    micro.setPosition(0.5);
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(46,-44.67))
                .lineTo(new Vector2d(35.82, -10.94))
                .addTemporalMarker(() -> {
                    targets = 0;
                    servoleft.setPosition(0.02);
                    servoright.setPosition(0.98);
                    micro.setPosition(1);
                })
                .lineTo(new Vector2d(60.59, -10.11))
                .build();
        TrajectorySequence closebardleftdelayed = drive.trajectorySequenceBuilder(new Pose2d(-40.86, -64.45, Math.toRadians(270.00)))
                .waitSeconds(7)
                .setReversed(true)
                .splineTo(new Vector2d(-37.88, -32.66), Math.toRadians(90.00))
                .splineTo(new Vector2d(-29 , -34.73), Math.toRadians(0.00))
                .lineTo(new Vector2d(-52.37, -30.03))
                .lineTo(new Vector2d(-52.37, -59.18))
                .lineTo(new Vector2d(38.28, -60.41))
                .addTemporalMarker(()->{
                    targets = rightHeight;
                    servoleft.setPosition(0.5);
                    servoright.setPosition(0.5);
                })
                .splineTo(new Vector2d(rightX, rightY), Math.toRadians(0.00))
                .addTemporalMarker(() ->{
                    micro.setPosition(0.5);
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(46,-44.67))
                .lineTo(new Vector2d(35.82, -10.94))
                .addTemporalMarker(() -> {
                    targets = 0;
                    servoleft.setPosition(0.02);
                    servoright.setPosition(0.98);
                    micro.setPosition(1);
                })
                .lineTo(new Vector2d(60.59, -10.11))
                .build();
        TrajectorySequence midbardright = drive.trajectorySequenceBuilder(new Pose2d(-40.39, -64.62, Math.toRadians(270.00)))
                .setReversed(true)
                .splineTo(new Vector2d(-36.53, -31.79), Math.toRadians(90.00))

                .lineTo(new Vector2d(-36.88, -37.05))
                .lineTo(new Vector2d(18.09, -36.35))
                .addTemporalMarker(()->{
                    targets = midHeight;
                    servoleft.setPosition(0.5);
                    servoright.setPosition(0.5);
                })
                .lineToLinearHeading(new Pose2d(midX, midY, Math.toRadians(180)))
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    micro.setPosition(0.5);
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(47.59, -36.53))
                .addTemporalMarker(() -> {
                    targets = 0;
                    servoleft.setPosition(0.02);
                    servoright.setPosition(0.98);
                    micro.setPosition(1);
                })
                .lineTo(new Vector2d(37.58, -64.80))
                .lineTo(new Vector2d(63.04, -63.57))
                .build();
        TrajectorySequence midbardrightdelayed = drive.trajectorySequenceBuilder(new Pose2d(-40.39, -64.62, Math.toRadians(270.00)))
                .waitSeconds(7)
                .setReversed(true)
                .splineTo(new Vector2d(-36.53, -31.79), Math.toRadians(90.00))

                .lineTo(new Vector2d(-36.88, -37.05))
                .lineTo(new Vector2d(18.09, -36.35))
                .addTemporalMarker(()->{
                    targets = midHeight;
                    servoleft.setPosition(0.5);
                    servoright.setPosition(0.5);
                })
                .lineToLinearHeading(new Pose2d(midX, midY, Math.toRadians(180)))
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    micro.setPosition(0.5);
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(47.59, -36.53))
                .addTemporalMarker(() -> {
                    targets = 0;
                    servoleft.setPosition(0.02);
                    servoright.setPosition(0.98);
                    micro.setPosition(1);
                })
                .lineTo(new Vector2d(37.58, -64.80))
                .lineTo(new Vector2d(63.04, -63.57))
                .build();
        TrajectorySequence midbardleftdelayed = drive.trajectorySequenceBuilder(new Pose2d(-40.39, -64.62, Math.toRadians(270.00)))
                .waitSeconds(7)
                .setReversed(true)
                .splineTo(new Vector2d(-36.53, -31.79), Math.toRadians(90.00))

                .lineTo(new Vector2d(-36.88, -37.05))
                .lineTo(new Vector2d(18.09, -36.35))
                .addTemporalMarker(()->{
                    targets = midHeight;
                    servoleft.setPosition(0.5);
                    servoright.setPosition(0.5);
                })
                .lineToLinearHeading(new Pose2d(midX, midY, Math.toRadians(180)))
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    micro.setPosition(0.5);
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(47.59, -36.53))
                .addTemporalMarker(() -> {
                    targets = 0;
                    servoleft.setPosition(0.02);
                    servoright.setPosition(0.98);
                    micro.setPosition(1);
                })
                .lineTo(new Vector2d(37.58, -10.80))
                .lineTo(new Vector2d(63.04, -10.57))
                .build();
        TrajectorySequence midbardleft = drive.trajectorySequenceBuilder(new Pose2d(-40.39, -64.62, Math.toRadians(270.00)))
                .setReversed(true)
                .splineTo(new Vector2d(-36.53, -31.79), Math.toRadians(90.00))

                .lineTo(new Vector2d(-36.88, -37.05))
                .lineTo(new Vector2d(18.09, -36.35))
                .addTemporalMarker(()->{
                    targets = midHeight;
                    servoleft.setPosition(0.5);
                    servoright.setPosition(0.5);
                })
                .lineToLinearHeading(new Pose2d(midX, midY, Math.toRadians(180)))
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    micro.setPosition(0.5);
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(47.59, -36.53))
                .addTemporalMarker(() -> {
                    targets = 0;
                    servoleft.setPosition(0.02);
                    servoright.setPosition(0.98);
                    micro.setPosition(1);
                })
                .lineTo(new Vector2d(37.58, -10.80))
                .lineTo(new Vector2d(63.04, -10.57))
                .build();

        TrajectorySequence farbardright = drive.trajectorySequenceBuilder(new Pose2d(-40.86, -64.45, Math.toRadians(270.00)))
                .setReversed(true)
                .splineTo(new Vector2d(-33.72, -33.54), Math.toRadians(90))
                .splineTo(new Vector2d(-41.80, -31.61), Math.toRadians(180))
                .setReversed(false)
                .lineTo(new Vector2d(-33.38, -33.89))
                .lineTo(new Vector2d(-43.55, -59.88))
                .lineTo(new Vector2d(34.95, -59.71))
//                .addTemporalMarker(() -> {
//                    micro.setPosition(0.5);
//                })
                //.waitSeconds(1)
                .addTemporalMarker(()->{
                    targets = leftHeight;
                    servoleft.setPosition(0.5);
                    servoright.setPosition(0.5);
                })
                .setReversed(true)
                .splineTo(new Vector2d(33.72, -33.19), Math.toRadians(0.00))
                .lineTo(new Vector2d(leftX, leftY))
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    micro.setPosition(0.5);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    targets = 0;
                    servoleft.setPosition(0.02);
                    servoright.setPosition(0.98);
                    micro.setPosition(1);
                })
                .lineTo(new Vector2d(47.77, -59.88))
                .lineTo(new Vector2d(65.50, -60.41))
                .build();
        TrajectorySequence farbardrightdelayed = drive.trajectorySequenceBuilder(new Pose2d(-40.86, -64.45, Math.toRadians(270.00)))
                .waitSeconds(7)
                .setReversed(true)
                .splineTo(new Vector2d(-33.72, -33.54), Math.toRadians(90))
                .splineTo(new Vector2d(-41.80, -31.61), Math.toRadians(180))
                .setReversed(false)
                .lineTo(new Vector2d(-33.38, -33.89))
                .lineTo(new Vector2d(-43.55, -59.88))
                .lineTo(new Vector2d(34.95, -59.71))
//                .addTemporalMarker(() -> {
//                    micro.setPosition(0.5);
//                })
                //.waitSeconds(1)
                .addTemporalMarker(()->{
                    targets = leftHeight;
                    servoleft.setPosition(0.5);
                    servoright.setPosition(0.5);
                })
                .setReversed(true)
                .splineTo(new Vector2d(33.72, -33.19), Math.toRadians(0.00))
                .lineTo(new Vector2d(leftX, leftY))
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    micro.setPosition(0.5);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    targets = 0;
                    servoleft.setPosition(0.02);
                    servoright.setPosition(0.98);
                    micro.setPosition(1);
                })
                .lineTo(new Vector2d(47.77, -59.88))
                .lineTo(new Vector2d(65.50, -60.41))
                .build();
        TrajectorySequence farbardleft = drive.trajectorySequenceBuilder(new Pose2d(-40.86, -64.45, Math.toRadians(270.00)))
                .setReversed(true)
                .splineTo(new Vector2d(-33.72, -33.54), Math.toRadians(90))
                .splineTo(new Vector2d(-41.80, -31.61), Math.toRadians(180))
                .setReversed(false)
                .lineTo(new Vector2d(-33.38, -33.89))
                .lineTo(new Vector2d(-43.55, -59.88))
                .lineTo(new Vector2d(34.95, -59.71))

                // .waitSeconds(1)
                .addTemporalMarker(()->{
                    targets = leftHeight;
                    servoleft.setPosition(0.5);
                    servoright.setPosition(0.5);
                })
                .setReversed(true)
                .splineTo(new Vector2d(33.72, -33.19), Math.toRadians(0.00))
                .lineTo(new Vector2d(leftX, leftY))
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    micro.setPosition(0.5);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    targets = 0;
                    servoleft.setPosition(0.02);
                    servoright.setPosition(0.98);
                    micro.setPosition(1);
                })
                .lineTo(new Vector2d(47.77, -10.88))
                .lineTo(new Vector2d(65.50, -10.41))
                .build();
        TrajectorySequence farbardleftdelayed = drive.trajectorySequenceBuilder(new Pose2d(-40.86, -64.45, Math.toRadians(270.00)))
                .waitSeconds(7)
                .setReversed(true)
                .splineTo(new Vector2d(-33.72, -33.54), Math.toRadians(90))
                .splineTo(new Vector2d(-41.80, -31.61), Math.toRadians(180))
                .setReversed(false)
                .lineTo(new Vector2d(-33.38, -33.89))
                .lineTo(new Vector2d(-43.55, -59.88))
                .lineTo(new Vector2d(34.95, -59.71))

                // .waitSeconds(1)
                .addTemporalMarker(()->{
                    targets = leftHeight;
                    servoleft.setPosition(0.5);
                    servoright.setPosition(0.5);
                })
                .setReversed(true)
                .splineTo(new Vector2d(33.72, -33.19), Math.toRadians(0.00))
                .lineTo(new Vector2d(leftX, leftY))
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    micro.setPosition(0.5);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    targets = 0;
                    servoleft.setPosition(0.02);
                    servoright.setPosition(0.98);
                    micro.setPosition(1);
                })
                .lineTo(new Vector2d(47.77, -10.88))
                .lineTo(new Vector2d(65.50, -10.41))
                .build();

        while (!isStarted() && !isStopRequested()) {
            if(gamepad1.dpad_left){
                parkposition = "left";
            }
            else if(gamepad1.dpad_right){
                parkposition = "right";
            }
            if(gamepad1.dpad_up){
                delayed = true;
            }
            else if (gamepad1.dpad_down){
                delayed = false;
            }


            telemetry.addLine("Ready!");
            telemetry.addData("Distance from Cone", cv.TSGPipeline.getDistanceFromCone());
            telemetry.addData("location", cv.TSGPipeline.coneCenterX);
            if (cv.TSGPipeline.coneCenterX > 50 && cv.TSGPipeline.coneCenterX < 400 && cv.TSGPipeline.getDistanceFromCone() < 200&&cv.TSGPipeline.pixel_height!=0) {
                telemetry.addData("TSG Position", "left");
                location = 1;
            } else if (cv.TSGPipeline.coneCenterX > 300 && cv.TSGPipeline.getDistanceFromCone() < 200&&cv.TSGPipeline.pixel_height!=0) {
                telemetry.addData("TSG Position", "middle");
                location =2;
            } else {
                telemetry.addData("TSG Position", "right");
                location = 3;
            }
            telemetry.addData("PARK POSITION", parkposition);
            telemetry.addData("DELAYED", delayed);

            telemetry.update();
        }



        waitForStart();
        switch (location){
            case 1:
                if(parkposition == "left") {
                    if(delayed == true){
                        drive.followTrajectorySequenceAsync(farbardleftdelayed);
                    }
                    else if(delayed == false){
                        drive.followTrajectorySequenceAsync(farbardleft);

                    }

                }
                else if(parkposition == "right"){
                    if(delayed == true){
                        drive.followTrajectorySequenceAsync(farbardrightdelayed);

                    }
                    else if(delayed == false){
                        drive.followTrajectorySequenceAsync(farbardright);

                    }
                }


                break;
            case 2:
                if(parkposition == "left") {
                    if(delayed == true){
                        drive.followTrajectorySequenceAsync(midbardleftdelayed);
                    }
                    else if(delayed == false){
                        drive.followTrajectorySequenceAsync(midbardleft);

                    }

                }
                else if(parkposition == "right"){
                    if(delayed == true){
                        drive.followTrajectorySequenceAsync(midbardrightdelayed);

                    }
                    else if(delayed == false){
                        drive.followTrajectorySequenceAsync(midbardright);

                    }
                }

                break;
            case 3:
                if(parkposition == "left") {
                    if(delayed == true){
                        drive.followTrajectorySequenceAsync(closebardleftdelayed);
                    }
                    else if(delayed == false){
                        drive.followTrajectorySequenceAsync(closebardleft);

                    }

                }
                else if(parkposition == "right"){
                    if(delayed == true){
                        drive.followTrajectorySequenceAsync(closebardrightdelayed);

                    }
                    else if(delayed == false){
                        drive.followTrajectorySequenceAsync(closebardright);

                    }
                }



                break;
        }
        while(opModeIsActive()){
            slidescontroller.setPID(ps, is, ds);
            int slidesleftpos = slidesleft.getCurrentPosition();
            double pidleft = slidescontroller.calculate(slidesleftpos, targets);
            double ffleft = Math.cos(Math.toRadians(targets / ticks_in_degrees)) * fs;

            int slidesrightpos = slidesright.getCurrentPosition();
            double pidright = slidescontroller.calculate(slidesrightpos, targets);
            double ffright = Math.cos(Math.toRadians(targets / ticks_in_degrees)) * fs;

            double powerleft = pidleft + ffleft;

            double powerright = pidright + ffright;
            slidesleft.setPower(powerleft);
            slidesright.setPower(powerright);
            drive.update();


        }
    }
}