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

@Autonomous(name = "badautoblue", preselectTeleOp = "teleop")
public class badautoblue extends LinearOpMode {
    public DcMotor intake;
    public Servo servoleft;
    public Servo servoright;
    public DcMotor slidesleft;
    public DcMotor slidesright;
    public Servo micro;
    public int location;
    private PIDController slidescontroller;
    public static double ps = 0.002, is = 0, ds = 0;
    public static double fs = 0.01;
    public int targets = 0;
    private final double ticks_in_degrees = 384 / 360.0;

    public double leftX = 58;
    public double leftY = 40.87;
    public double midX = 58;
    public double midY = 37.75;
    public double rightX = 58;
    public double rightY = 29.5;
    public int rightHeight = 1300;
    public int midHeight = 1300;
    public int leftHeight = 1300;
    public void runOpMode(){
        CVMaster cv = new CVMaster(this);
        cv.observeCone();

        telemetry.addData("tsg location", cv.conePipeline.coneCenterX);
        telemetry.addData("size", cv.conePipeline.distance);
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

        drive.setPoseEstimate(new Pose2d(16.86, 64.92, Math.toRadians(90.00)));

        TrajectorySequence closeboard = drive.trajectorySequenceBuilder(new Pose2d(17.05, 64.55, Math.toRadians(90.00)))
                .setReversed(true)
                .splineTo(new Vector2d(17.60, 28.02), Math.toRadians(0.00))
                .setReversed(false)
                .lineTo(new Vector2d(9, 36.02))
                .addTemporalMarker(() -> {
                    targets = leftHeight;
                    servoleft.setPosition(0.5);
                    servoright.setPosition(0.5);
                })
                .setReversed(true)
                .splineTo(new Vector2d(12.53, 46.11), Math.toRadians(85.54))
                .splineTo(new Vector2d(leftX, leftY), Math.toRadians(0.00))
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

                .lineTo(new Vector2d(50.66, 63.55))
                .lineTo(new Vector2d(63.55, 63.36))
                .build();

        TrajectorySequence farboard = drive.trajectorySequenceBuilder(new Pose2d(16.86, 64.92, Math.toRadians(90.00)))
                .setReversed(true)
                .splineTo(new Vector2d(4, 38.67), Math.toRadians(-135.00))
                .setReversed(false)
                .lineTo(new Vector2d(21.33, 51.51))
                                .addTemporalMarker(() -> {
                                    targets = rightHeight;
                                    servoleft.setPosition(0.5);
                                    servoright.setPosition(0.5);
                                })
                .setReversed(true)
                .splineTo(new Vector2d(rightX, rightY), Math.toRadians(0.00))
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
                .lineTo(new Vector2d(50.66, 63.55))
                .lineTo(new Vector2d(63.55, 63.36))
                .build();
        TrajectorySequence midbard = drive.trajectorySequenceBuilder(new Pose2d(16.89, 64.64, Math.toRadians(90.00)))
                .setReversed(true)
                .splineTo(new Vector2d(12.17, 32.27), Math.toRadians(270.00))
                .setReversed(false)
                .lineTo(new Vector2d(12.36, 50.30))
                .setReversed(true)
                                .addTemporalMarker(() -> {
                                    targets = midHeight;
                                    servoleft.setPosition(0.5);
                                    servoright.setPosition(0.5);
                                })
                .splineTo(new Vector2d(midX, midY), Math.toRadians(0.00))
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
                .lineTo(new Vector2d(50.66, 63.55))
                .lineTo(new Vector2d(63.55, 63.36))


                .build();

        while (!isStarted() && !isStopRequested()) {


            telemetry.addLine("Ready!");
            telemetry.addData("Distance from Cone", cv.conePipeline.getDistanceFromCone());
            telemetry.addData("location", cv.conePipeline.coneCenterX);
            if (cv.conePipeline.coneCenterX > 50 && cv.conePipeline.coneCenterX < 400 && cv.conePipeline.getDistanceFromCone() < 100&&cv.conePipeline.pixel_height!=0) {
                telemetry.addData("TSG Position", "left");
                location = 1;
            } else if (cv.conePipeline.coneCenterX > 300 && cv.conePipeline.getDistanceFromCone() < 100&&cv.conePipeline.pixel_height!=0) {
                telemetry.addData("TSG Position", "middle");
                location =2;
            } else {
                telemetry.addData("TSG Position", "right");
                location = 3;
            }

            telemetry.update();
        }



        waitForStart();
        switch (location){
            case 1:
                drive.followTrajectorySequenceAsync(closeboard);
                break;
            case 2:
                drive.followTrajectorySequenceAsync(midbard);
                break;
            case 3:
                drive.followTrajectorySequenceAsync(farboard);
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