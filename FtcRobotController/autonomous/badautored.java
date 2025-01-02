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

@Autonomous(name = "badautored", preselectTeleOp = "teleop")
public class badautored extends LinearOpMode {
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
    public double rightX = 55;
    public double rightY = -45;
    public double midX = 55;
    public double midY = -38.5;
    public double leftX = 55;
    public double leftY = -33.75;

    public int rightHeight = 1400;
    public int midHeight = 1350;
    public int leftHeight = 1400;
    public void runOpMode() {
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


        drive.setPoseEstimate(new Pose2d(16.89, -64.26, Math.toRadians(270.00)));
        TrajectorySequence untitled2 = drive.trajectorySequenceBuilder(new Pose2d(16.89, -64.26, Math.toRadians(270.00)))
                .setReversed(true)
                .splineTo(new Vector2d(17.60, -36.02), Math.toRadians(0.00))
                .setReversed(false)
                .lineTo(new Vector2d(9, -36.02))
                .addTemporalMarker(() -> {
                    targets = leftHeight;
                    servoleft.setPosition(0.5);
                    servoright.setPosition(0.5);
                })
                .setReversed(true)
                .splineTo(new Vector2d(12.53, -46.11), Math.toRadians(-85.54))
                .splineTo(new Vector2d(rightX, rightY), Math.toRadians(0.00))
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    micro.setPosition(0.5);
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(52.66,-44.75))
                .addTemporalMarker(() -> {
                    targets = 0;
                    servoleft.setPosition(0.02);
                    servoright.setPosition(0.98);
                    micro.setPosition(1);
                })
                .lineTo(new Vector2d(52.66, -63.55))
                .lineTo(new Vector2d(63.55, -63.36))
                .build();
        TrajectorySequence untitled1 = drive.trajectorySequenceBuilder(new Pose2d(16.89, -64.26, Math.toRadians(270.00)))
                .setReversed(true)
                .splineTo(new Vector2d(7, -35.67), Math.toRadians(135.00))
                .setReversed(false)
                .lineTo(new Vector2d(21.33, -51.51))
                .addTemporalMarker(() -> {
                    targets = rightHeight;
                    servoleft.setPosition(0.5);
                    servoright.setPosition(0.5);
                })
                .setReversed(true)
                .splineTo(new Vector2d(leftX, leftY), Math.toRadians(0.00))
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    micro.setPosition(0.5);
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(52.66,-32.09))
                .addTemporalMarker(() -> {
                    targets = 0;
                    servoleft.setPosition(0.02);
                    servoright.setPosition(0.98);
                    micro.setPosition(1);
                })
                .lineTo(new Vector2d(52.66, -63.55))
                .lineTo(new Vector2d(63.55, -63.36))
                .build();

        TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(16.89, -64.26, Math.toRadians(270.00)))
                .setReversed(true)
                .splineTo(new Vector2d(15.17, -32.27), Math.toRadians(90.00))
                .setReversed(false)
                .lineTo(new Vector2d(12.36, -50.30))
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
                .lineTo(new Vector2d(52.66,-38.95))
                .addTemporalMarker(() -> {
                    targets = 0;
                    servoleft.setPosition(0.02);
                    servoright.setPosition(0.98);
                    micro.setPosition(1);
                })
                .lineTo(new Vector2d(52.66, -63.55))
                .lineTo(new Vector2d(63.55, -63.36))


                .build();

        while (!isStarted() && !isStopRequested()) {


            telemetry.addLine("Ready!");
            telemetry.addData("Distance from Cone", cv.TSGPipeline.getDistanceFromCone());
            telemetry.addData("location", cv.TSGPipeline.coneCenterX);
            if (cv.TSGPipeline.coneCenterX > 50 && cv.TSGPipeline.coneCenterX < 400 && cv.TSGPipeline.getDistanceFromCone() < 200&&cv.TSGPipeline.pixel_height!=0) {
                telemetry.addData("TSG Position", "middle");
                location = 2;
            } else if (cv.TSGPipeline.coneCenterX > 450 && cv.TSGPipeline.getDistanceFromCone() < 200&&cv.TSGPipeline.pixel_height!=0) {
                telemetry.addData("TSG Position", "right");
                location = 3;
            } else {
                telemetry.addData("TSG Position", "left");
                location = 1;
            }

            telemetry.update();
        }



        waitForStart();
        switch (location){
            case 1:
                drive.followTrajectorySequenceAsync(untitled1);
                break;
            case 2:
                drive.followTrajectorySequenceAsync(untitled0);
                break;
            case 3:
                drive.followTrajectorySequenceAsync(untitled2);
                break;
        }
        cv.stopCamera();
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