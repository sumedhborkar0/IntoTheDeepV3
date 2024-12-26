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
    public class BadAuto extends LinearOpMode {
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
        public double power = 0;
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
            servoleft.setPosition(0.068);
            servoright.setPosition(0.932);
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
            TrajectorySequence untitled2 =  drive.trajectorySequenceBuilder(new Pose2d(-39.86, -68.31, Math.toRadians(90.00)))
                    .setReversed(true)
                    .splineTo(new Vector2d(-40.39, -30.20), Math.toRadians(180.00))
                    .lineTo(new Vector2d(-31.79, -30.38))
                    .lineTo(new Vector2d(-35.30, -50.58))
                    .lineToSplineHeading(new Pose2d(-60.94, -37.05, Math.toRadians(0.00)))
                    .addTemporalMarker(() -> {

                    })
                    .lineTo(new Vector2d(-29.68, -59.18))
                    .lineTo(new Vector2d(33.01, -57.07))
                    .splineTo(new Vector2d(53.21, -36.18), Math.toRadians(0.00))
                    .lineTo(new Vector2d(49.35, -36.00))
                    .lineTo(new Vector2d(43.73, -45.66))
                    .lineTo(new Vector2d(61.29, -63.75))
                    .build();

            TrajectorySequence untitled1 = drive.trajectorySequenceBuilder(new Pose2d(-40.74, -64.98, Math.toRadians(270.00)))
                    .setReversed(true)
                    .splineTo(new Vector2d(-36.00, -31.79), Math.toRadians(90.00))
                    .lineTo(new Vector2d(-36.70, -41.09))
                    .splineToLinearHeading(new Pose2d(-59.71, -36.35, Math.toRadians(180.00)), Math.toRadians(180.00))
                    .lineTo(new Vector2d(52.86, -36.88))
                    .lineTo(new Vector2d(46.89, -36.70))
                    .lineTo(new Vector2d(37.76, -59.71))
                    .lineTo(new Vector2d(63.04, -59.53))
                    .build();

            TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(-41.44, -66.73, Math.toRadians(270.00)))
                    .setReversed(true)
                    .splineTo(new Vector2d(-31.08, -30.38), Math.toRadians(0.00))
                    .lineTo(new Vector2d(-61.64, -36.18))
                    .lineTo(new Vector2d(-26.87, -59.88))
                    .lineTo(new Vector2d(29.33, -60.06))
                    .splineTo(new Vector2d(53.03, -40.74), Math.toRadians(0.00))
                    .lineTo(new Vector2d(48.12, -41.27))
                    .lineTo(new Vector2d(43.20, -62.52))
                    .lineTo(new Vector2d(60.59, -61.64))


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
            while(opModeIsActive() && !isStopRequested()){
                power = 0.6;
            }



            waitForStart();
            switch (location){
                case 2:
                    drive.followTrajectorySequenceAsync(untitled1);
                    break;
                case 1:
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

