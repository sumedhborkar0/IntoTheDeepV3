package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.controller.PIDController;

// NOT COMPLETE
@Autonomous(name = "AutoV3", preselectTeleOp = "TeleopV2")
public class AutoV3 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        DcMotor slidesRight = hardwareMap.dcMotor.get("slidesRight");
        DcMotor slidesLeft = hardwareMap.dcMotor.get("slidesLeft");
        DcMotor intake = hardwareMap.dcMotor.get("intake");

        Servo rightIntakeServo = hardwareMap.servo.get("rightServo");
        Servo leftIntakeServo = hardwareMap.servo.get("leftServo");
        Servo fourBarRight = hardwareMap.servo.get("fourBarRight");
        Servo fourBarleft = hardwareMap.servo.get("fourBarLeft");
        Servo intakeAngle = hardwareMap.servo.get("intakeAngle");

        Servo rightWrist = hardwareMap.servo.get("rightWrist");
        Servo leftWrist = hardwareMap.servo.get("leftWrist");
        Servo armServo = hardwareMap.servo.get("armMotor");
        Servo clawServo = hardwareMap.servo.get("clawMotor");

        double fs = 0.01, ticks_in_degrees = 1;
        PIDController controller = new PIDController(0.004,0,0);

        //REVERSE + INITIATE ENCODERS
        rightIntakeServo.setDirection(Servo.Direction.REVERSE);
        slidesLeft.setDirection(DcMotor.Direction.REVERSE);
        fourBarRight.setDirection(Servo.Direction.REVERSE);

        slidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //INIATE MOTOR POSITIONS
        leftIntakeServo.setPosition(0.5);
        rightIntakeServo.setPosition(0.5);
        rightWrist.setPosition(0.5);
        leftWrist.setPosition(0.5);
        armServo.setPosition(0.5);
        clawServo.setPosition(0.5);

        intakeAngle.setPosition(0);
        intake.setPower(0);
        fourBarRight.setPosition(0.25);
        fourBarleft.setPosition(0.25);

        double targets = 100;

        Action PIDAction = new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                int slidesleftpos = slidesLeft.getCurrentPosition();
                double pidleft = controller.calculate(slidesleftpos, targets);
                double ffleft = Math.cos(Math.toRadians(targets / ticks_in_degrees)) * fs;

                int slidesrightpos = slidesRight.getCurrentPosition();
                double pidright = controller.calculate(slidesrightpos, targets);
                double ffright = Math.cos(Math.toRadians(targets / ticks_in_degrees)) * fs;

                double powerleft = pidleft + ffleft;

                double powerright = pidright + ffright;

                slidesLeft.setPower(powerleft);
                slidesRight.setPower(powerright);
                return true;
            }
        };


        waitForStart();

        if (isStopRequested()) return;
/*
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-60, 0, Math.toRadians(90.00)))
                .lineToY(-45)
                .lineToX(-35)
                .lineToY(-60)
                .lineToY(-45)
                .lineToX(-60)
                .lineToY(0)
                .build());
*/

        Actions.runBlocking(new ParallelAction(
                PIDAction,
                new SequentialAction(

                )

        ));




    }

}