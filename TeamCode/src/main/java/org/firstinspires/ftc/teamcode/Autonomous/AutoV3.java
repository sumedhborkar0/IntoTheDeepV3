package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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

import java.time.Instant;

// NOT COMPLETE
@Autonomous(name = "AutoV3", preselectTeleOp = "TeleopV2")
public class AutoV3 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-60,39,0));

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
        double intakeAngle_IntakingPos = 0.535;
        double intakeAngle_RetractedPos = 0;
        double extendoRetractedPos = 0.48;
        double extendoExtendedPos = 0.61;
        double fourBarRetractedPos = 0.15;
        double fourBarExtendedPos = 0.85;

        double armInitPos = 0.35;
        double armPickupPos = 0.25;
        double armDropPos = 0.5494;
        double wristInitPos = 0.8;
        double wristPickupPos = 0.975;
        double wristDropPos = 0.1994;
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
        clawServo.setPosition(clawClosePos);

        intakeAngle.setPosition(intakeAngle_RetractedPos);
        intake.setPower(stopPower);
        fourBarRight.setPosition(fourBarRetractedPos);
        fourBarleft.setPosition(fourBarRetractedPos);



        myPIDAction pidAction = new myPIDAction(hardwareMap);

        InstantAction highLevelAction = new InstantAction(
                () -> {
                    pidAction.targets = highLevel;
                }
        );
        InstantAction groundLevelAction = new InstantAction(
                () -> {
                    pidAction.targets = groundLevel;
                }
        );
        InstantAction closeClaw = new InstantAction(
                () -> {
                    clawServo.setPosition(clawClosePos);
                }
        );
        InstantAction openClaw = new InstantAction(
                () -> {
                    clawServo.setPosition(clawOpenPos);
                }
        );
        InstantAction armDropAction = new InstantAction(
                () -> {
                    armServo.setPosition(armDropPos);
                }
        );
        InstantAction armInitAction = new InstantAction(
                () -> {
                    armServo.setPosition(armInitPos);
                }
        );
        InstantAction wristDropAction = new InstantAction(
                () -> {
                    leftWrist.setPosition(wristDropPos);
                    rightWrist.setPosition(wristDropPos);
                }
        );
        InstantAction wristInitAction = new InstantAction(
                () -> {
                    leftWrist.setPosition(wristInitPos);
                    rightWrist.setPosition(wristInitPos);
                }
        );
        InstantAction intakeExtendedAction = new InstantAction(
                () -> {
                    intakeAngle.setPosition(intakeAngle_IntakingPos);
                    fourBarRight.setPosition(fourBarExtendedPos);
                    fourBarleft.setPosition(fourBarExtendedPos);
                }
        );
        InstantAction intakeRetractedAction = new InstantAction(
                () -> {
                    intakeAngle.setPosition(intakeAngle_RetractedPos);
                    fourBarRight.setPosition(fourBarRetractedPos);
                    fourBarleft.setPosition(fourBarRetractedPos);
                }
        );

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new ParallelAction(
                pidAction.calcPID(),
                new SequentialAction(
                        new ParallelAction(
                                drive.actionBuilder(new Pose2d(-63,39,0))
                                    .strafeToLinearHeading(new Vector2d(-52,56.5), -45)
                                    .build(),
                                highLevelAction),
                        new ParallelAction(
                                armDropAction,
                                wristDropAction
                        ),
                        new SleepAction(0.2),
                        openClaw,
                        new ParallelAction(
                                armInitAction,
                                wristInitAction

                        ),
                        new SleepAction(0.1),
                        groundLevelAction,
                        new ParallelAction(
                                drive.actionBuilder(new Pose2d(-52, 56.5, 45))
                                        .strafeToLinearHeading(new Vector2d(-52, 48), 0)
                                        .build(),
                                intakeExtendedAction

                        ),
                        drive.actionBuilder(new Pose2d(-52, 48, 0))
                                .strafeToLinearHeading(new Vector2d(-37, 48), 0)
                                .build()




                )
                )


        );




    }

}