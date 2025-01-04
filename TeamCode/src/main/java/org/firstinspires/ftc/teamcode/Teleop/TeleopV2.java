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
        Servo rightIntakeServo = hardwareMap.servo.get("rightServo");
        Servo leftIntakeServo = hardwareMap.servo.get("leftServo");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        Servo fourBarRight =hardwareMap.servo.get("fourBarRight");
        Servo fourBarleft =hardwareMap.servo.get("fourBarLeft");
        double fs = 0.01, ticks_in_degrees = 1;
        PIDController controller = new PIDController(0.004,0,0);


        //REVERSE + INITIATE ENCODERS
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
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

        intake.setPower(0);
        fourBarRight.setPosition(0.3);
        fourBarleft.setPosition(0.3);


        double targets = 100;

        // CURRENT STATES
        boolean going_down = false;
        boolean scissor_extended = false;
        boolean intake_reversed = false;
        boolean intake_running = false;

        // BUTTON RELEASES
        boolean gamepad1_rightBumperReleased = true;
        boolean gamepad1_leftBumperReleased = true;
        boolean gamepad1_dPadDownReleased = true;
        boolean gamepad1_dPadUpReleased = true;


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

        // GAMEPAD 1 CONTROLS
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
                rightIntakeServo.setPosition(0.7);
                leftIntakeServo.setPosition(0.7);
                fourBarRight.setPosition(0.85);
                fourBarleft.setPosition(0.85);
                intake.setPower(0.5);
                scissor_extended = true;
                intake_running = false;
                gamepad1_rightBumperReleased = false;
            }
            // inits scissor lift, extends 4bar
            if (gamepad1.left_bumper && !intake_running && gamepad1_leftBumperReleased) {
                rightIntakeServo.setPosition(0.5);
                leftIntakeServo.setPosition(0.5);
                fourBarRight.setPosition(0.85);
                fourBarleft.setPosition(0.85);
                intake.setPower(0.5);
                intake_running = true;
                scissor_extended = false;
                gamepad1_leftBumperReleased = false;
            }
            // inits scissor lift, inits 4bar
            if ((gamepad1.left_bumper && gamepad1_leftBumperReleased && intake_running) || (gamepad1.right_bumper && gamepad1_rightBumperReleased && scissor_extended)) {
                rightIntakeServo.setPosition(0.5);
                leftIntakeServo.setPosition(0.5);
                fourBarRight.setPosition(0.3);
                fourBarleft.setPosition(0.3);
                intake.setPower(0);
                intake_running = false;
                scissor_extended = false;
                gamepad1_rightBumperReleased = false;
                gamepad1_leftBumperReleased = false;
            }

            if (gamepad1.dpad_down && gamepad1_dPadDownReleased) {
                double currPos = fourBarRight.getPosition();
                fourBarRight.setPosition(currPos - 0.1);
                fourBarleft.setPosition(currPos - 0.1);
                gamepad1_dPadDownReleased = false;
            }
            else if (gamepad1.dpad_up && gamepad1_dPadUpReleased) {
                double currPos = fourBarRight.getPosition();
                fourBarRight.setPosition(currPos + 0.1);
                fourBarleft.setPosition(currPos + 0.1);
                gamepad1_dPadUpReleased = false;
            }

            if (gamepad1.left_trigger != 0 && !intake_reversed) {
                intake.setPower(-.5);
                intake_reversed = true;
            }
            else if (gamepad1.left_trigger != 0 && intake_reversed) {
                intake.setPower(0);
                intake_reversed = false;
            }

        // GAMEPAD 2 CONTROLS ////////////////////////////////////////
            //SLIDES PID
            if (gamepad2.left_bumper) {
                targets = 0;
                going_down = true;
            }
            if (going_down && (slidesRight.getCurrentPosition() < 40 || slidesLeft.getCurrentPosition() < 40)){
                going_down = false;
                targets = 100;
            }
            if (gamepad2.a){
                targets = 0;
                going_down = true;
                //MOVE ARM
            }
            else if (gamepad2.b) {
                targets = 1070;
                // Move Arm for hanging the specimen
            }
            else if (gamepad2.x) {
                targets = 1070;
                // Move arm for putting the sample in the basket
            }
            else if (gamepad2.y) {
                targets = 2800;
                // Move arm for putting the sample in the top basket
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

            telemetry.addData("LeftSlidePower", powerleft);
            telemetry.addData("RightSlidePower", powerright);
            telemetry.update();

            telemetry.addData("LeftSlide", slidesLeft.getCurrentPosition());
            telemetry.addData("RightSlide", slidesRight.getCurrentPosition());
            telemetry.update();

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



        }

    }
}