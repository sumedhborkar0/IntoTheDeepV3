package org.firstinspires.ftc.teamcode.Teleop;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;


import com.acmerobotics.roadrunner.HolonomicController;

import org.firstinspires.ftc.teamcode.controller.PIDController;


@TeleOp (name = "TeleopV2")
public class TeleopV2 extends LinearOpMode {
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
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightIntakeServo.setDirection(Servo.Direction.REVERSE);
        slidesLeft.setDirection(DcMotor.Direction.REVERSE);
        fourBarRight.setDirection(Servo.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        rightWrist.setDirection(Servo.Direction.REVERSE);

        slidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //POWERS & POSITIONS
        double intakePower = 0.75;


        //INIATE MOTOR POSITIONS
        leftIntakeServo.setPosition(0.48);
        rightIntakeServo.setPosition(0.48);
        rightWrist.setPosition(0.5);
        leftWrist.setPosition(0.5);
        armServo.setPosition(0.5);
        clawServo.setPosition(0.4);

        intakeAngle.setPosition(0);
        intake.setPower(0);
        fourBarRight.setPosition(0.25);
        fourBarleft.setPosition(0.25);


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
        boolean gamepad1_leftBumperReleased = true;
        boolean gamepad1_dPadDownReleased = true;
        boolean gamepad1_dPadUpReleased = true;
        boolean gamepad1_leftTriggerReleased = true;
        boolean gamepad2_xReleased = true;
        boolean gamepad2_yReleased = true;
//TESTING
        boolean gamepad1_yReleased = true;
        boolean gamepad1_aReleased = true;



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

        // GAMEPAD 1 CONTROLS

             //ARM MOTOR POSITION TESTING
            if (gamepad1_aReleased && gamepad1.a) {
                double currPosition = leftWrist.getPosition();
                leftWrist.setPosition(currPosition - 0.05);
                rightWrist.setPosition(currPosition-.05);
                gamepad1_aReleased = false;
            }
            if (gamepad1_yReleased && gamepad1.y) {
                double currPosition = leftWrist.getPosition();
                leftWrist.setPosition(currPosition + 0.05);
                rightWrist.setPosition(currPosition+.05);
                gamepad1_yReleased = false;
            }


            //MECANUM DRIVE
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);


        // SCISSOR LIFT + INTAKE ///////////////

            // extends scissor lift, extends 4bar
            if (gamepad1.right_bumper && !scissor_extended && gamepad1_rightBumperReleased) {
                rightIntakeServo.setPosition(0.61);
                leftIntakeServo.setPosition(0.61);
                fourBarRight.setPosition(0.85);
                fourBarleft.setPosition(0.85);
                intakeAngle.setPosition(0.5);
                intake.setPower(intakePower);
                scissor_extended = true;
                intake_running = false;
                gamepad1_rightBumperReleased = false;
            }
            // inits scissor lift, extends 4bar
            if (gamepad1.left_bumper && !intake_running && gamepad1_leftBumperReleased) {
                rightIntakeServo.setPosition(0.48);
                leftIntakeServo.setPosition(0.48);
                fourBarRight.setPosition(0.85);
                fourBarleft.setPosition(0.85);
                intakeAngle.setPosition(0.5);
                intake.setPower(intakePower);
                intake_running = true;
                scissor_extended = false;
                gamepad1_leftBumperReleased = false;
            }
            // inits scissor lift, inits 4bar
            if ((gamepad1.left_bumper && gamepad1_leftBumperReleased && intake_running) || (gamepad1.right_bumper && gamepad1_rightBumperReleased && scissor_extended)) {
                rightIntakeServo.setPosition(0.48);
                leftIntakeServo.setPosition(0.48);
                fourBarRight.setPosition(0.25);
                fourBarleft.setPosition(0.25);
                intakeAngle.setPosition(0);
                intake.setPower(0.15);
                intake_running = false;
                scissor_extended = false;
                gamepad1_rightBumperReleased = false;
                gamepad1_leftBumperReleased = false;
            }

            if (gamepad1.dpad_down && gamepad1_dPadDownReleased) {
                double currPos = fourBarRight.getPosition();
                fourBarRight.setPosition(currPos+ 0.05);
                fourBarleft.setPosition(currPos + 0.05);
                gamepad1_dPadDownReleased = false;
            }
            else if (gamepad1.dpad_up && gamepad1_dPadUpReleased) {
                double currPos = fourBarleft.getPosition();
                fourBarleft.setPosition(currPos + 0.1);
                fourBarRight.setPosition(currPos + 0.1);
                gamepad1_dPadUpReleased = false;
            }



            if (gamepad1.left_trigger != 0 && !intake_reversed && gamepad1_leftTriggerReleased) {
                intake.setPower(-1);
                intake_reversed = true;
                gamepad1_leftTriggerReleased = false;
            }
            else if (gamepad1.left_trigger != 0 && intake_reversed && gamepad1_leftTriggerReleased) {
                intake.setPower(0);
                intake_reversed = false;
                gamepad1_leftTriggerReleased = false;
            }

        // GAMEPAD 2 CONTROLS ////////////////////////////////////////
            //SLIDES PID
            if (gamepad2.left_bumper) {
                armServo.setPosition(0.5);
                clawServo.setPosition(0.4);
                leftWrist.setPosition(0.5);
                rightWrist.setPosition(0.5);
                Thread.sleep(250);
                targets = 0;
                going_down = true;
                clawReset = true;
            }
            if (going_down && (slidesRight.getCurrentPosition() < 40 || slidesLeft.getCurrentPosition() < 40)){
                going_down = false;
                targets = 100;
            }
            if (((gamepad2.x && gamepad2_xReleased) || (gamepad2.y && gamepad2_yReleased)) && clawReset) {
                clawServo.setPosition(0.4);
                leftWrist.setPosition(0.95);
                rightWrist.setPosition(0.95);
                armServo.setPosition(0.25);
                clawReset = false;
                gamepad2_xReleased = false;
                gamepad2_yReleased = false;
            }
            if (gamepad2.a){
                targets = 0;
                going_down = true;
                //CLOSE CLAW, MOVE ARM
            }
            else if (gamepad2.b) {
                //CLOSE CLAW
                targets = 1200;
                // Move Arm for hanging the specimen
            }
            else if (gamepad2.x && !extended_to_basket && gamepad2_xReleased && !clawReset) {
                clawServo.setPosition(0.6);
                Thread.sleep(100);
                targets = 1200;
                intake.setPower(0);
                extended_to_basket = true;
                gamepad2_xReleased = false;
                clawReset = false;
                // Move arm for putting the sample in the basket
            }
            else if (gamepad2.y && !extended_to_basket && gamepad2_yReleased && !clawReset) {
                clawServo.setPosition(0.6);
                Thread.sleep(100);
                targets = 2800;
                intake.setPower(0);
                extended_to_basket = true;
                gamepad2_yReleased = false;
                clawReset = false;
                // Move arm for putting the sample in the top basket // 0.8
            }
            else if (((gamepad2.x && gamepad2_xReleased) || (gamepad2.y && gamepad2_yReleased)) && extended_to_basket ) {
                armServo.setPosition(0.7);
                leftWrist.setPosition(0.28);
                rightWrist.setPosition(0.28);
                gamepad2_yReleased = false;
                gamepad2_xReleased = false;
                extended_to_basket = false;
            }
            if (gamepad2.right_bumper) {
                clawServo.setPosition(0.4);
            }




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
            if (!gamepad2.y) {
                gamepad2_yReleased = true;
            }





        }

    }
}