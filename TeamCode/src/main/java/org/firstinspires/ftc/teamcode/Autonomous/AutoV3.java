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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.controller.PIDController;
import org.firstinspires.ftc.teamcode.controller.PIDFController;

// NOT COMPLETE
@Autonomous(name = "AutoV3", preselectTeleOp = "TeleopV2")
public class AutoV3 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        DcMotor intake = hardwareMap.dcMotor.get("intake");

        Servo rightExtendoServo = hardwareMap.servo.get("rightServo");
        Servo leftExtendoServo = hardwareMap.servo.get("leftServo");
        Servo fourBarRight = hardwareMap.servo.get("fourBarRight");
        Servo fourBarleft = hardwareMap.servo.get("fourBarLeft");
        Servo intakeAngle = hardwareMap.servo.get("intakeAngle");

        Servo rightWrist = hardwareMap.servo.get("rightWrist");
        Servo leftWrist = hardwareMap.servo.get("leftWrist");
        Servo armServo = hardwareMap.servo.get("armMotor");
        Servo clawServo = hardwareMap.servo.get("clawMotor");



        //REVERSE + INITIATE ENCODERS

        rightExtendoServo.setDirection(Servo.Direction.REVERSE);

        fourBarRight.setDirection(Servo.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        rightWrist.setDirection(Servo.Direction.REVERSE);





        //POWERS & POSITIONS //////////////////////////////////
        double stopPower = 0;

        double intakePower = 0.75;
        double intakeSlowPower = 0.15;
        double intakeAngle_IntakingPos = 0.5;
        double intakeAngle_RetractedPos = 0;
        double extendoRetractedPos = 0.48;
        double extendoExtendedPos = 0.61;
        double fourBarRetractedPos = 0.25;
        double fourBarExtendedPos = 0.85;

        double armInitPos = 0.5;
        double armPickupPos = 0.25;
        double armDropPos = 0.7;
        double wristInitPos = 0.5;
        double wristPickupPos = 0.95;
        double wristDropPos = 0.28;
        double clawOpenPos = 0.4;
        double clawClosePos = 0.6;

        double groundLevel = 0;
        double initLevel = 100;
        double midLevel = 1200;
        double highLevel = 2800;


        //INIATE MOTOR POSITIONS
        leftExtendoServo.setPosition(extendoRetractedPos);
        rightExtendoServo.setPosition(extendoRetractedPos);
        rightWrist.setPosition(wristInitPos);
        leftWrist.setPosition(wristInitPos);
        armServo.setPosition(armInitPos);
        clawServo.setPosition(clawOpenPos);

        intakeAngle.setPosition(intakeAngle_RetractedPos);
        intake.setPower(stopPower);
        fourBarRight.setPosition(fourBarRetractedPos);
        fourBarleft.setPosition(fourBarRetractedPos);



        myPIDAction pidAction = new myPIDAction(hardwareMap);

        InstantAction testHeightChange = new InstantAction(
                () -> {
                    pidAction.targets = 500;
                }
        );
        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new ParallelAction(
                pidAction.calcPID(),
                new SequentialAction(

                        testHeightChange
                )

        ));




    }

}