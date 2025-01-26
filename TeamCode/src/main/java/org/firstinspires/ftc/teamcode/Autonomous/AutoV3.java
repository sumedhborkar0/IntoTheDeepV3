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

        // INITIATE MOTORS

        DcMotor slidesRight = hardwareMap.dcMotor.get("slidesRight");
        DcMotor slidesLeft = hardwareMap.dcMotor.get("slidesLeft");
        DcMotor intake = hardwareMap.dcMotor.get("intake");

        Servo rightExtendoServo = hardwareMap.servo.get("rightExtendoServo");
        Servo leftExtendoServo = hardwareMap.servo.get("leftExtendoServo");
        Servo intakeRight = hardwareMap.servo.get("intakeRight");
        Servo intakeLeft = hardwareMap.servo.get("intakeLeft");
        Servo intakeWrist = hardwareMap.servo.get("intakeWrist");

        Servo rightArm = hardwareMap.servo.get("rightArm");
        Servo leftArm = hardwareMap.servo.get("leftArm");
        Servo clawWrist = hardwareMap.servo.get("clawWrist");
        Servo clawServo = hardwareMap.servo.get("clawMotor");

        double kp = 0.004, ki = 0, kd = 0, kf = 0.0000007;
        PIDFController controller = new PIDFController(kp, ki, kd, kf);

        //REVERSE + INITIATE ENCODERS
        slidesRight.setDirection(DcMotor.Direction.REVERSE);
        intakeRight.setDirection(Servo.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        rightArm.setDirection(Servo.Direction.REVERSE);
        rightExtendoServo.setDirection(Servo.Direction.REVERSE);

        slidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //POWERS & POSITIONS //////////////////////////////////
        double stopPower = 0;

        double intakePower = 1;
        double intakeSlowPower = 0;
        double intakeWrist_IntakingPos = 0.665;
        double intakeWrist_RetractedPos = 0.09;
        double extendoRetractedPos = 0.45  ;
        double extendoExtendedPos = 0.675;
        double fourBarRetractedPos = 0.3194;
        double fourBarExtendedPos = 0.9061; // maybe 0.75

        double armInitPos = 0.65;
        double armPickupPos = 0.6;
        double armDropPos = 0.35;
        double armVerticalPos = 0.5;
        double wristInitPos = 0.35;
        double wristPickupPos = 0.25 ;
        double wristVerticalPos = 0.5;
        double wristDropPos = 0.6494;
        double clawOpenPos = 0.55;
        double clawClosePos = 0.35;

        double groundLevel = 0;
        double midLevel = 1250;
        double highLevel = 2600;



        double startMovingArmBackDistFromTarget = 600;


        //INIATE MOTOR POSITIONS
        leftExtendoServo.setPosition(extendoRetractedPos);
        rightExtendoServo.setPosition(extendoRetractedPos);
        rightArm.setPosition(armVerticalPos);
        leftArm.setPosition(armVerticalPos);
        clawWrist.setPosition(wristVerticalPos);
        clawServo.setPosition(clawClosePos);

        intakeWrist.setPosition(intakeWrist_RetractedPos);
        intake.setPower(stopPower);
        intakeRight.setPosition(fourBarRetractedPos);
        intakeLeft.setPosition(fourBarRetractedPos);



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
                    clawWrist.setPosition(armDropPos);
                }
        );
        InstantAction armInitAction = new InstantAction(
                () -> {
                    clawWrist.setPosition(armInitPos);
                }
        );
        InstantAction wristDropAction = new InstantAction(
                () -> {
                    leftArm.setPosition(wristDropPos);
                    rightArm.setPosition(wristDropPos);
                }
        );
        InstantAction wristInitAction = new InstantAction(
                () -> {
                    leftArm.setPosition(wristInitPos);
                    rightArm.setPosition(wristInitPos);
                }
        );
        InstantAction intakeExtendedAction = new InstantAction(
                () -> {
                    intake.setPower(intakePower);
                    intakeRight.setPosition(fourBarExtendedPos);
                    intakeLeft.setPosition(fourBarExtendedPos);
                    new SleepAction(0.2);
                    intakeWrist.setPosition(intakeWrist_IntakingPos);
                }
        );
        InstantAction intakeRetractedAction = new InstantAction(
                () -> {
                    intakeWrist.setPosition(intakeWrist_RetractedPos);
                    intakeRight.setPosition(fourBarRetractedPos);
                    intakeLeft.setPosition(fourBarRetractedPos);
                    intake.setPower(intakeSlowPower);
                }
        );
        InstantAction armToPickupPosAction = new InstantAction(
                () -> {
                    clawWrist.setPosition(armPickupPos);
                    leftArm.setPosition(wristPickupPos);
                    rightArm.setPosition(wristPickupPos);

                }
        );
        InstantAction intakeWristPutDown = new InstantAction(
                () -> {
                    intakeWrist.setPosition(intakeWrist_IntakingPos + 0.19);
                    intakeLeft.setPosition(0.875);
                    intakeRight.setPosition(0.875);
                }
        );
        InstantAction armParkPosAction = new InstantAction(
                () -> {
                    clawWrist.setPosition(0.55);
                }
        );


        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new ParallelAction(
                pidAction.calcPID(),
                new SequentialAction(
                        new ParallelAction(
                                drive.actionBuilder(new Pose2d(-60,39,0))
                                        .strafeTo(new Vector2d(-53,55))
                                        .turn(Math.toRadians(-90))
                                        .build(),
                                highLevelAction),
                        new ParallelAction(
                                armDropAction,
                                wristDropAction
                        ),
                        new SleepAction(0.8),
                        openClaw,
                        new SleepAction(0.4),
                        new ParallelAction(
                                armInitAction,
                                wristInitAction

                        ),
                        new SleepAction(0.5),
                        groundLevelAction
                        /*
                        new ParallelAction(
                                drive.actionBuilder(new Pose2d(-53, 55, -45))
                                        .strafeToLinearHeading(new Vector2d(-52, 48), 0)
                                        .build(),
                                intakeExtendedAction

                        ),
                        drive.actionBuilder(new Pose2d(-52, 48, 0))
                                .strafeToLinearHeading(new Vector2d(-31, 54), 0)
                                .build(),
                        new SleepAction(.8),
                        intakeRetractedAction,
                        new SleepAction(1.1),
                        new ParallelAction(
                                drive.actionBuilder(new Pose2d(-31,54,0))
                                        .strafeToLinearHeading(new Vector2d(-53,56), -45)
                                        .build(),
                                new SequentialAction(
                                        armToPickupPosAction,
                                        new SleepAction(0.5),
                                        closeClaw
                                )

                        ),
                        highLevelAction,
                        new SleepAction(1.2),
                        new ParallelAction(
                                armDropAction,
                                wristDropAction
                        ),
                        new SleepAction(1),
                        openClaw,

                        // SAMPLE TWO
                        new SleepAction(0.4),
                        new ParallelAction(
                                armInitAction,
                                wristInitAction

                        ),
                        new SleepAction(0.5),
                        groundLevelAction//,
                        new ParallelAction(
                                drive.actionBuilder(new Pose2d(-53, 56, -45))
                                        .strafeToLinearHeading(new Vector2d(-52, 56), 0)
                                        .build(),
                                intakeExtendedAction

                        ),
                        drive.actionBuilder(new Pose2d(-52, 56, 0))
                                .strafeToLinearHeading(new Vector2d(-33, 63), 0)
                                .build(),
                        new SleepAction(.5),
                        new SequentialAction(
                                intakeWristPutDown,
                                drive.actionBuilder(new Pose2d(-33, 63, 0))
                                        .strafeToLinearHeading(new Vector2d(-31, 63), 0)
                                        .build()
                        ),
                        new SleepAction(.8),
                        intakeRetractedAction,
                        new SleepAction(1.1),
                        new ParallelAction(
                                drive.actionBuilder(new Pose2d(-31,63,0))
                                        .strafeToLinearHeading(new Vector2d(-53,55), -45)
                                        .build(),
                                new SequentialAction(
                                        armToPickupPosAction,
                                        new SleepAction(0.5),
                                        closeClaw
                                )

                        ),
                        highLevelAction,
                        new SleepAction(1.2),
                        new ParallelAction(
                                armDropAction,
                                wristDropAction
                        ),
                        new SleepAction(1),
                        openClaw,

                        // SAMPLE 3
                        new SleepAction(0.4),
                        new ParallelAction(
                                armInitAction,
                                wristInitAction

                        ),
                        new SleepAction(0.5),
                        groundLevelAction,
                        new ParallelAction(
                                drive.actionBuilder(new Pose2d(-53, 55, -45))
                                        .strafeToLinearHeading(new Vector2d(-29, 45), 0)
                                        .build(),
                                intakeExtendedAction

                        ),
                        drive.actionBuilder(new Pose2d(-29, 45, 0))
                                .strafeToLinearHeading(new Vector2d(-29, 45), -90)
                                .build(),
                        new SleepAction(.5),
                        new SequentialAction(
                                intakeWristPutDown,
                                drive.actionBuilder(new Pose2d(-29, 45, -90))
                                        .strafeToLinearHeading(new Vector2d(-29, 52), -90)
                                        .build()
                        ),
                        new SleepAction(.8),
                        intakeRetractedAction,
                        new SleepAction(1.1),
                        new ParallelAction(
                                drive.actionBuilder(new Pose2d(-29,52,-90))
                                        .strafeToLinearHeading(new Vector2d(-53,55), -45)
                                        .build(),
                                new SequentialAction(
                                        armToPickupPosAction,
                                        new SleepAction(0.5),
                                        closeClaw
                                )

                        ),
                        highLevelAction,
                        new SleepAction(1.2),
                        new ParallelAction(
                                armDropAction,
                                wristDropAction
                        ),
                        new SleepAction(1),
                        openClaw,

                        // PARK
                        new SleepAction(0.4),
                        new ParallelAction(
                                armInitAction,
                                wristInitAction

                        ),
                        new SleepAction(0.5),
                        groundLevelAction,
                        drive.actionBuilder(new Pose2d(-53,55,-45))
                                .strafeToLinearHeading(new Vector2d(0,26), -90)
                                .build(),
                        armParkPosAction*/






                )
                )


        );




    }

}