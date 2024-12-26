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

@Autonomous(name = "badautobluefar", preselectTeleOp = "teleop")
public class badautobluefar extends LinearOpMode {
    public DcMotor intake;
    public Servo servoleft;
    public Servo servoright;
    public DcMotor slidesleft;
    public DcMotor slidesright;
    public Servo micro;
    public String parkposition = "left";
    public boolean delayed = false;
    public int location;
    private PIDController slidescontroller;
    public static double ps = 0.002, is = 0, ds = 0;
    public static double fs = 0.01;
    public int targets = 0;
    private final double ticks_in_degrees = 384 / 360.0;
    public double leftX = 55;
    public double leftY = 40.87;
    public double midX = 54.5;
    public double midY = 37.75;
    public double rightX = 55;
    public double rightY = 29.5;
    public int rightHeight = 1500;
    public int midHeight = 1500;
    public int leftHeight = 1600;
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
        drive.setPoseEstimate(new Pose2d(-40.86, 64.45, Math.toRadians(90.00)));
        TrajectorySequence closebardrightdelayed = drive.trajectorySequenceBuilder(new Pose2d(-40.86, 64.45, Math.toRadians(90.00)))
                .waitSeconds(7)
                .setReversed(true)
                .splineTo(new Vector2d(-30.49, 32.41), Math.toRadians(-45.00))
                .setReversed(false)
                .lineTo(new Vector2d(-52.84, 58.43))
                .setReversed(true)
                .splineTo(new Vector2d(-30.17, 58.91), Math.toRadians(14.78))
                .splineTo(new Vector2d(3.19, 60.35), Math.toRadians(0.00))
                .addTemporalMarker(() -> {
                    targets = leftHeight;
                    servoleft.setPosition(0.5);
                    servoright.setPosition(0.5);
                })
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
                .setReversed(false)
                .lineTo(new Vector2d(45.66, 37.95))
                .setReversed(true)

                .lineTo(new Vector2d(32.66, 14.55))
                .lineTo(new Vector2d(63.55, 14.36))
                .build();
        TrajectorySequence closebardright = drive.trajectorySequenceBuilder(new Pose2d(-40.86, 64.45, Math.toRadians(90.00)))
                .setReversed(true)
                .splineTo(new Vector2d(-30.49, 32.41), Math.toRadians(-45.00))
                .setReversed(false)
                .lineTo(new Vector2d(-52.84, 58.43))
                .setReversed(true)
                .splineTo(new Vector2d(-30.17, 58.91), Math.toRadians(14.78))
                .splineTo(new Vector2d(3.19, 60.35), Math.toRadians(0.00))
                .addTemporalMarker(() -> {
                    targets = leftHeight;
                    servoleft.setPosition(0.5);
                    servoright.setPosition(0.5);
                })
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
                .setReversed(false)
                .lineTo(new Vector2d(45.66, 37.95))
                .setReversed(true)

                .lineTo(new Vector2d(32.66, 14.55))
                .lineTo(new Vector2d(63.55, 14.36))
                .build();

        TrajectorySequence closebardleftdelayed = drive.trajectorySequenceBuilder(new Pose2d(-40.86, 64.45, Math.toRadians(90.00)))
                .waitSeconds(7)
                .setReversed(true)
                .splineTo(new Vector2d(-30.49, 32.41), Math.toRadians(-45.00))
                .setReversed(false)
                .lineTo(new Vector2d(-52.84, 58.43))
                .setReversed(true)
                .splineTo(new Vector2d(-30.17, 58.91), Math.toRadians(14.78))
                .splineTo(new Vector2d(3.19, 60.35), Math.toRadians(0.00))
                .addTemporalMarker(() -> {
                    targets = leftHeight;
                    servoleft.setPosition(0.5);
                    servoright.setPosition(0.5);
                })
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
                .setReversed(false)
                .lineTo(new Vector2d(45.66, 37.95))
                .setReversed(true)

                .lineTo(new Vector2d(32.66, 63.55))
                .lineTo(new Vector2d(63.55, 63.36))
                .build();
        TrajectorySequence closebardleft = drive.trajectorySequenceBuilder(new Pose2d(-40.86, 64.45, Math.toRadians(90.00)))
                .setReversed(true)
                .splineTo(new Vector2d(-30.49, 32.41), Math.toRadians(-45.00))
                .setReversed(false)
                .lineTo(new Vector2d(-52.84, 58.43))
                .setReversed(true)
                .splineTo(new Vector2d(-30.17, 58.91), Math.toRadians(14.78))
                .splineTo(new Vector2d(3.19, 60.35), Math.toRadians(0.00))
                .addTemporalMarker(() -> {
                    targets = leftHeight;
                    servoleft.setPosition(0.5);
                    servoright.setPosition(0.5);
                })
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
                .setReversed(false)
                .lineTo(new Vector2d(45.66, 37.95))
                .setReversed(true)

                .lineTo(new Vector2d(32.66, 63.55))
                .lineTo(new Vector2d(63.55, 63.36))
                .build();
        TrajectorySequence midbardrightdelayed = drive.trajectorySequenceBuilder(new Pose2d(-40.39, 64.62, Math.toRadians(90.00)))
                .waitSeconds(7)
                .setReversed(true)
                .splineTo(new Vector2d(-36.53, 31.79), Math.toRadians(270.00))

                .lineTo(new Vector2d(-36.88, 37.05))
                .lineTo(new Vector2d(18.09, 36.35))
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
                .lineTo(new Vector2d(47.59, 36.53))
                .addTemporalMarker(() -> {
                    targets = 0;
                    servoleft.setPosition(0.02);
                    servoright.setPosition(0.98);
                    micro.setPosition(1);
                })
                .lineTo(new Vector2d(37.58, 14.80))
                .lineTo(new Vector2d(63.04, 14.57))
                .build();
        TrajectorySequence midbardright = drive.trajectorySequenceBuilder(new Pose2d(-40.39, 64.62, Math.toRadians(90.00)))
                .setReversed(true)
                .splineTo(new Vector2d(-36.53, 31.79), Math.toRadians(270.00))

                .lineTo(new Vector2d(-36.88, 37.05))
                .lineTo(new Vector2d(18.09, 36.35))
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
                .lineTo(new Vector2d(47.59, 36.53))
                .addTemporalMarker(() -> {
                    targets = 0;
                    servoleft.setPosition(0.02);
                    servoright.setPosition(0.98);
                    micro.setPosition(1);
                })
                .lineTo(new Vector2d(37.58, 14.80))
                .lineTo(new Vector2d(63.04, 14.57))
                .build();
        TrajectorySequence midbardleftdelayed = drive.trajectorySequenceBuilder(new Pose2d(-40.39, 64.62, Math.toRadians(90.00)))
                .waitSeconds(7)
                .setReversed(true)
                .splineTo(new Vector2d(-36.53, 31.79), Math.toRadians(270.00))

                .lineTo(new Vector2d(-36.88, 37.05))
                .lineTo(new Vector2d(18.09, 36.35))
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
                .lineTo(new Vector2d(47.59, 36.53))
                .addTemporalMarker(() -> {
                    targets = 0;
                    servoleft.setPosition(0.02);
                    servoright.setPosition(0.98);
                    micro.setPosition(1);
                })
                .lineTo(new Vector2d(37.58, 64.80))
                .lineTo(new Vector2d(63.04, 63.57))
                .build();




        TrajectorySequence midbardleft = drive.trajectorySequenceBuilder(new Pose2d(-40.39, 64.62, Math.toRadians(90.00)))
                .setReversed(true)
                .splineTo(new Vector2d(-36.53, 31.79), Math.toRadians(270.00))

                .lineTo(new Vector2d(-36.88, 37.05))
                .lineTo(new Vector2d(18.09, 36.35))
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
                .lineTo(new Vector2d(47.59, 36.53))
                .addTemporalMarker(() -> {
                    targets = 0;
                    servoleft.setPosition(0.02);
                    servoright.setPosition(0.98);
                    micro.setPosition(1);
                })
                .lineTo(new Vector2d(37.58, 64.80))
                .lineTo(new Vector2d(63.04, 63.57))
                .build();
        TrajectorySequence farbardrightdelayed = drive.trajectorySequenceBuilder(new Pose2d(-40.86, 64.45, Math.toRadians(90.00)))
                .waitSeconds(7)
                .setReversed(true)
                .splineTo(new Vector2d(-42.20, 34.36), Math.toRadians(180.00))
                .setReversed(false)
                .lineTo(new Vector2d(-20.85, 34.82))
                .setReversed(true)
                .splineTo(new Vector2d(-37.77, 44.85), Math.toRadians(90.00))
                .splineTo(new Vector2d(5.70, 57.41), Math.toRadians(0.00))
                .addTemporalMarker(() -> {
                    targets = rightHeight;
                    servoleft.setPosition(0.5);
                    servoright.setPosition(0.5);
                })
                .splineTo(new Vector2d(rightX, rightY), Math.toRadians(0.00))
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    micro.setPosition(0.5);
                })
                .waitSeconds(1)
                .setReversed(false)
                .lineTo(new Vector2d(45.66, 30.09))



                .addTemporalMarker(() -> {
                    targets = 0;
                    servoleft.setPosition(0.02);
                    servoright.setPosition(0.98);
                    micro.setPosition(1);
                })

                .setReversed(true)

                .lineTo(new Vector2d(28.66, 14.55))
                .lineTo(new Vector2d(63.55, 14.36))
                .build();
        TrajectorySequence farbardright = drive.trajectorySequenceBuilder(new Pose2d(-40.86, 64.45, Math.toRadians(90.00)))
                .setReversed(true)
                .splineTo(new Vector2d(-42.20, 34.36), Math.toRadians(180.00))
                .setReversed(false)
                .lineTo(new Vector2d(-20.85, 34.82))
                .setReversed(true)
                .splineTo(new Vector2d(-37.77, 44.85), Math.toRadians(90.00))
                .splineTo(new Vector2d(5.70, 57.41), Math.toRadians(0.00))
                .addTemporalMarker(() -> {
                    targets = rightHeight;
                    servoleft.setPosition(0.5);
                    servoright.setPosition(0.5);
                })
                .splineTo(new Vector2d(rightX, rightY), Math.toRadians(0.00))
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    micro.setPosition(0.5);
                })
                .waitSeconds(1)
                .setReversed(false)
                .lineTo(new Vector2d(45.66, 30.09))



                .addTemporalMarker(() -> {
                    targets = 0;
                    servoleft.setPosition(0.02);
                    servoright.setPosition(0.98);
                    micro.setPosition(1);
                })

                .setReversed(true)

                .lineTo(new Vector2d(28.66, 14.55))
                .lineTo(new Vector2d(63.55, 14.36))
                .build();
        TrajectorySequence farbardleftdelayed = drive.trajectorySequenceBuilder(new Pose2d(-40.86, 64.45, Math.toRadians(90.00)))
                .waitSeconds(7)
                .setReversed(true)
                .splineTo(new Vector2d(-42.20, 34.36), Math.toRadians(180.00))
                .setReversed(false)
                .lineTo(new Vector2d(-20.85, 34.82))
                .setReversed(true)
                .splineTo(new Vector2d(-37.77, 44.85), Math.toRadians(90.00))
                .splineTo(new Vector2d(5.70, 57.41), Math.toRadians(0.00))
                .addTemporalMarker(() -> {
                    targets = rightHeight;
                    servoleft.setPosition(0.5);
                    servoright.setPosition(0.5);
                })
                .splineTo(new Vector2d(rightX, rightY), Math.toRadians(0.00))
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    micro.setPosition(0.5);
                })
                .waitSeconds(1)
                .setReversed(false)
                .lineTo(new Vector2d(45.66, 30.09))



                .addTemporalMarker(() -> {
                    targets = 0;
                    servoleft.setPosition(0.02);
                    servoright.setPosition(0.98);
                    micro.setPosition(1);
                })

                .setReversed(true)

                .lineTo(new Vector2d(28.66, 63.55))
                .lineTo(new Vector2d(63.55, 63.36))
                .build();
        TrajectorySequence farbardleft = drive.trajectorySequenceBuilder(new Pose2d(-40.86, 64.45, Math.toRadians(90.00)))
                .setReversed(true)
                .splineTo(new Vector2d(-42.20, 34.36), Math.toRadians(180.00))
                .setReversed(false)
                .lineTo(new Vector2d(-20.85, 34.82))
                .setReversed(true)
                .splineTo(new Vector2d(-37.77, 44.85), Math.toRadians(90.00))
                .splineTo(new Vector2d(5.70, 57.41), Math.toRadians(0.00))
                .addTemporalMarker(() -> {
                    targets = rightHeight;
                    servoleft.setPosition(0.5);
                    servoright.setPosition(0.5);
                })
                .splineTo(new Vector2d(rightX, rightY), Math.toRadians(0.00))
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    micro.setPosition(0.5);
                })
                .waitSeconds(1)
                .setReversed(false)
                .lineTo(new Vector2d(45.66, 30.09))



                .addTemporalMarker(() -> {
                    targets = 0;
                    servoleft.setPosition(0.02);
                    servoright.setPosition(0.98);
                    micro.setPosition(1);
                })

                .setReversed(true)

                .lineTo(new Vector2d(28.66, 63.55))
                .lineTo(new Vector2d(63.55, 63.36))
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
            telemetry.addData("Distance from Cone", cv.conePipeline.getDistanceFromCone());
            telemetry.addData("location", cv.conePipeline.coneCenterX);
            if (cv.conePipeline.coneCenterX > 50 && cv.conePipeline.coneCenterX < 400 && cv.conePipeline.getDistanceFromCone() < 100&&cv.conePipeline.pixel_height!=0) {
                telemetry.addData("TSG Position", "middle");
                location = 2;
            } else if (cv.conePipeline.coneCenterX > 300 && cv.conePipeline.getDistanceFromCone() < 100&&cv.conePipeline.pixel_height!=0) {
                telemetry.addData("TSG Position", "right");
                location =3;
            } else {
                telemetry.addData("TSG Position", "left");
                location = 1;
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