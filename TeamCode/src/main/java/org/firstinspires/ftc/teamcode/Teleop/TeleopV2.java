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
        // Declare our motors
        // Make sure your ID's match your configuration


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


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightIntakeServo.setDirection(Servo.Direction.REVERSE);
        slidesLeft.setDirection(DcMotor.Direction.REVERSE);
        fourBarRight.setDirection(Servo.Direction.REVERSE);

        slidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
/////////////////////////////////////////////////////////////
        //INITIALIZATIONS
        leftIntakeServo.setPosition(0.5);
        rightIntakeServo.setPosition(0.5);
        double targets = 100;
        intake.setPower(0);
        fourBarRight.setPosition(0.3);
        fourBarleft.setPosition(0.3);
/////////////////////////////////////////////////////////////
        boolean goingDown = false;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

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


            // SCISSOR LIFT
            if (gamepad1.dpad_up) {
                fourBarRight.setPosition(0.8);
                fourBarleft.setPosition(0.8);
                rightIntakeServo.setPosition(0.7);
                leftIntakeServo.setPosition(0.7);
            }
            if (gamepad1.dpad_down) {
                rightIntakeServo.setPosition(0.5);
                leftIntakeServo.setPosition(0.5);
                fourBarRight.setPosition(0.3);
                fourBarleft.setPosition(0.3);
            }

            //SLIDES PID
            if (gamepad2.a) {
                targets = 0;
                goingDown = true;
            }
            else if (gamepad2.y){
                targets = 2800;
            }
            else if (gamepad2.b){
                targets = 1070;
                goingDown = true;
            }

            if (goingDown && (slidesRight.getCurrentPosition() < 50 || slidesLeft.getCurrentPosition() < 50)){
                goingDown = false;
                targets = 100;
            }
            if(gamepad1.right_bumper){
                intake.setPower(0.5);
            }
            else if(gamepad1.left_bumper){
                intake.setPower(-0.5);
            }
            else if(gamepad1.a){
                intake.setPower(0);
            }
            if(gamepad1.x){
                fourBarRight.setPosition(0.8);
                fourBarleft.setPosition(0.8);
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


        }

    }
}