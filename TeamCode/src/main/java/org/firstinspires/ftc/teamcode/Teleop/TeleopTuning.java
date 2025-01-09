package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controller.PIDFController;


@TeleOp (name = "TeleopV2")
public class TeleopTuning extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // INITIATE MOTORS
        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor leftBack = hardwareMap.dcMotor.get("leftBack");
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
        DcMotor rightBack = hardwareMap.dcMotor.get("rightBack");

        DcMotor slidesRight = hardwareMap.dcMotor.get("slidesRight");
        DcMotor slidesLeft = hardwareMap.dcMotor.get("slidesLeft");
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
        
        double kp = 0.004, ki = 0, kd = 0, kf = 0.0000007;
        PIDFController controller = new PIDFController(kp, ki, kd, kf);


        //REVERSE + INITIATE ENCODERS
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        slidesLeft.setDirection(DcMotor.Direction.REVERSE);
        fourBarRight.setDirection(Servo.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        rightWrist.setDirection(Servo.Direction.REVERSE);
        rightExtendoServo.setDirection(Servo.Direction.REVERSE);

        slidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //POWERS & POSITIONS //////////////////////////////////
        double stopPower = 0;

        double intakePower = 0.75;
        double intakeSlowPower = 0.15;
        double intakeAngle_IntakingPos = 0.6;
        double intakeAngle_RetractedPos = 0;
        double extendoRetractedPos = 0.4;
        double extendoExtendedPos = 0.7;
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


        double targets = 100;

        // CURRENT STATES
        boolean going_down = false;
        boolean scissor_extended = false;
        boolean intake_reversed = false;
        boolean intake_running = false;
        boolean extended_to_basket = false;
        boolean clawReset = true;

        // BUTTON RELEASES
        boolean gamepad1_rightBumperReleased = true;
        boolean gamepad1_rightTriggerReleased = true;
        boolean gamepad1_leftBumperReleased = true;
        boolean gamepad1_dPadDownReleased = true;
        boolean gamepad1_dPadUpReleased = true;
        boolean gamepad1_leftTriggerReleased = true;
        boolean gamepad2_xReleased = true;
        boolean gamepad2_yReleased = true;
        boolean gamepad1_xReleased = true;
        boolean gamepad1_bReleased = true;
//TESTING
        boolean gamepad1_yReleased = true;
        boolean gamepad1_aReleased = true;
        boolean gamepad1_dPadLeftReleased = true;
        boolean gamepad1_dPadRightReleased = true;



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

        // GAMEPAD 1 CONTROLS

             //ARM MOTOR POSITION TESTING
            if (gamepad1_aReleased && gamepad1.a) {
                double currPosition = leftWrist.getPosition();
                leftWrist.setPosition(currPosition - 0.05);
                rightWrist.setPosition(currPosition- 0.05);
                gamepad1_aReleased = false;
            }
            if (gamepad1_yReleased && gamepad1.y) {
                double currPosition = leftWrist.getPosition();
                leftWrist.setPosition(currPosition + 0.05);
                rightWrist.setPosition(currPosition+0.05);
                gamepad1_yReleased = false;
            }
            if (gamepad1.x && gamepad1_xReleased) {
                double currPosition = armServo.getPosition();
                armServo.setPosition(currPosition-0.05);
                gamepad1_xReleased = false;
            }
            if (gamepad1.b && gamepad1_bReleased) {
                double currPosition = armServo.getPosition();
                armServo.setPosition(currPosition+0.05);
                gamepad1_bReleased = false;
            }
            if (gamepad1_dPadDownReleased && gamepad1.dpad_down) {
                double currPosition = fourBarRight.getPosition();
                fourBarleft.setPosition(currPosition -0.05);
                fourBarRight.setPosition(currPosition-0.05);
            }
            if (gamepad1_dPadUpReleased && gamepad1.dpad_up) {
                double currPosition = fourBarRight.getPosition();
                fourBarleft.setPosition(currPosition +0.05);
                fourBarRight.setPosition(currPosition+0.05);
            }
            if (gamepad1_dPadLeftReleased && gamepad1.dpad_left) {
                double currPosition = intakeAngle.getPosition();
                intakeAngle.setPosition(currPosition - 0.05);
                gamepad1_dPadLeftReleased = false;
            }
            if (gamepad1_dPadRightReleased && gamepad1.dpad_right) {
                double currPosition = intakeAngle.getPosition();
                intakeAngle.setPosition(currPosition + 0.05);
                gamepad1_dPadRightReleased = false;
            }




            // CHECK IF BUTTONS RELEASED
            if (!gamepad1.right_bumper) {
                gamepad1_rightBumperReleased = true;
            }
            if (!gamepad1.left_bumper) {
                gamepad1_leftBumperReleased = true;
            }
            if (!gamepad1.dpad_up) {
                gamepad1_dPadUpReleased = true;
            }
            if (!gamepad1.dpad_down) {
                gamepad1_dPadDownReleased = true;
            }
            if (!gamepad1.dpad_right) {
                gamepad1_dPadRightReleased = true;
            }
            if (!gamepad1.dpad_left) {
                gamepad1_dPadLeftReleased = true;
            }
            if (gamepad1.left_trigger == 0) {
                gamepad1_leftTriggerReleased = true;
            }

            //ARM MOTOR POSITION TESTING
            if(!gamepad1.a){
                gamepad1_aReleased = true;
            }
            if(!gamepad1.y){
                gamepad1_yReleased = true;
            }
            if (!gamepad2.x) {
                gamepad2_xReleased = true;
            }
            if (!gamepad1.x) {
                gamepad1_xReleased = true;
            }
            if (!gamepad1.b) {
                gamepad1_bReleased = true;
            }
            if (!gamepad2.y) {
                gamepad2_yReleased = true;
            }
            if (gamepad1.right_trigger == 0) {
                gamepad1_rightTriggerReleased = true;
            }

            telemetry.addData("Arm Pos", armServo.getPosition());
            telemetry.addData("Wrist Pos", leftWrist.getPosition());
            telemetry.addData("Four Bar Pos", fourBarleft.getPosition());
            telemetry.addData("Intake Angle", intakeAngle.getPosition());
            telemetry.update();



        }

    }
}