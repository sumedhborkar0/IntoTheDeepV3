package org.firstinspires.ftc.teamcode.Teleop;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;


import com.acmerobotics.roadrunner.HolonomicController;

import org.firstinspires.ftc.teamcode.controller.PIDController;
import org.firstinspires.ftc.teamcode.controller.PIDFController;


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

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //POWERS & POSITIONS //////////////////////////////////
        double stopPower = 0;

        double intakePower = 0.71;
        double intakeSlowPower = 0.15;
        double intakeAngle_IntakingPos = 0.54;
        double intakeAngle_RetractedPos = 0.07;
        double extendoRetractedPos = 0.43  ;
        double extendoExtendedPos = 0.6;
        double fourBarRetractedPos = 0.15;
        double fourBarExtendedPos = 0.7917; // maybe 0.75

        double armInitPos = 0.6;
        double armPickupPos = 0.45;
        double armDropPos = 0.85;
        double armVerticalPos = 0.75;
        double wristInitPos = 0.8;
        double wristPickupPos = 0.9;
        double wristVerticalPos = 0.45;
        double wristDropPos = 0.0;
        double clawOpenPos = 0.4;
        double clawClosePos = 0.6;

        double groundLevel = 0;
        double initLevel = 100;
        double midLevel = 1250;
        double highLevel = 2750;

        double startMovingArmBackDistFromTarget = 600;


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


        double targets = groundLevel;

        // CURRENT STATES
        boolean going_down = false;
        boolean scissor_extended = false;
        boolean intake_reversed = false;
        boolean intake_running = false;
        boolean extended_to_basket = false;
        boolean clawReset = true;
        boolean armWristInited = true;
        boolean wristInPickupPos = false;
        boolean clawJustClosedOnSample = false;
        boolean clawClosed = false;
        boolean slidesGoingToMid = false;
        boolean slidesGoingtoHigh = false;
        boolean atDropPos = false;
        boolean justMovedToInitArm = false;

        // BUTTON RELEASES
        boolean gamepad1_rightBumperReleased = true;
        boolean gamepad1_rightTriggerReleased = true;
        boolean gamepad1_leftBumperReleased = true;
        boolean gamepad1_dPadDownReleased = true;
        boolean gamepad1_dPadUpReleased = true;
        boolean gamepad1_dPadRightReleased = true;
        boolean gamepad1_dPadLeftReleased = true;
        boolean gamepad1_leftTriggerReleased = true;


        boolean gamepad2_xReleased = true;
        boolean gamepad2_yReleased = true;
        boolean gamepad2_leftBumperReleased = true;
        boolean gamepad2_rightBumperReleased = true;
        boolean gamepad2_rightTriggerReleased = true;
        boolean gamepad2_leftTriggerReleased = true;
        boolean gamepad2_dPadLeftReleased = true;
        boolean gamepad2_dPadRightReleased = true;
        boolean gamepad2_dPadDownReleased = true;
        boolean gamepad2_dPadUpReleased = true;
// TIMES
        double wristInPickupPosTime = 0;
        double clawClosedTime = 0;
        double justMovedToDropPosTime = 0;
        double justMovedToInitArmTime = 0;



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // GAMEPAD 1 CONTROLS


            //ARM MOTOR POSITION TESTING


            //MECANUM DRIVE + SLIDES PIDF
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

            int slidesleftpos = slidesLeft.getCurrentPosition();
            double powerLeft = controller.calculate(slidesleftpos, targets);

            int slidesrightpos = slidesRight.getCurrentPosition();
            double powerRight = controller.calculate(slidesrightpos, targets);

            slidesLeft.setPower(powerLeft);
            slidesRight.setPower(powerRight);

            // SCISSOR LIFT + INTAKE ///////////////

            // extends scissor lift, extends 4bar
            if (gamepad1.right_bumper && !scissor_extended && gamepad1_rightBumperReleased) {
                intakeAngle.setPosition(intakeAngle_IntakingPos);
                rightExtendoServo.setPosition(extendoExtendedPos);
                leftExtendoServo.setPosition(extendoExtendedPos);
                fourBarRight.setPosition(fourBarExtendedPos);
                fourBarleft.setPosition(fourBarExtendedPos);
                intake.setPower(intakePower);
                scissor_extended = true;
                intake_running = false;
                gamepad1_rightBumperReleased = false;
            }
            // inits scissor lift, extends 4bar
            if (gamepad1.left_bumper && !intake_running && gamepad1_leftBumperReleased) {
                intakeAngle.setPosition(intakeAngle_IntakingPos);
                rightExtendoServo.setPosition(extendoRetractedPos);
                leftExtendoServo.setPosition(extendoRetractedPos);
                fourBarRight.setPosition(fourBarExtendedPos);
                fourBarleft.setPosition(fourBarExtendedPos);

                intake.setPower(intakePower);
                intake_running = true;
                scissor_extended = false;
                gamepad1_leftBumperReleased = false;
            }
            // inits scissor lift, inits 4bar
            if ((gamepad1.left_bumper && gamepad1_leftBumperReleased && intake_running) || (gamepad1.right_bumper && gamepad1_rightBumperReleased && scissor_extended)) {
                fourBarRight.setPosition(fourBarRetractedPos);
                fourBarleft.setPosition(fourBarRetractedPos);
                rightExtendoServo.setPosition(extendoRetractedPos);
                leftExtendoServo.setPosition(extendoRetractedPos);
                intakeAngle.setPosition(intakeAngle_RetractedPos);
                intake.setPower(intakeSlowPower);
                intake_running = false;
                scissor_extended = false;
                gamepad1_rightBumperReleased = false;
                gamepad1_leftBumperReleased = false;
            }

            if (gamepad1.left_trigger != 0 && gamepad1_leftTriggerReleased) {
                intake.setPower(-1);
                gamepad1_leftTriggerReleased = false;
            } else if (gamepad1.left_trigger != 0 && intake.getPower() <= -0.5 && gamepad1_leftTriggerReleased) {
                intake.setPower(stopPower);
                gamepad1_leftTriggerReleased = false;
            }

            // GAMEPAD 2 CONTROLS ////////////////////////////////////////
            //SLIDES PID
            if (gamepad2.left_bumper && gamepad2_leftBumperReleased && !atDropPos) {
                leftWrist.setPosition(wristInitPos);
                rightWrist.setPosition(wristInitPos);
                armServo.setPosition(armInitPos);
                clawServo.setPosition(clawOpenPos);
                targets = groundLevel;
                armWristInited = true;
                gamepad2_leftBumperReleased = false;
            }
            else if (gamepad2.left_bumper && gamepad2_leftBumperReleased && atDropPos) {
                leftWrist.setPosition(wristInitPos);
                rightWrist.setPosition(wristInitPos);
                armServo.setPosition(armInitPos);
                justMovedToInitArmTime = System.currentTimeMillis();
                justMovedToInitArm = true;
                clawServo.setPosition(clawOpenPos);
                atDropPos = false;
                gamepad2_leftBumperReleased = false;

            }
            if (justMovedToInitArm && System.currentTimeMillis() - justMovedToInitArmTime >= 150) {
                targets = groundLevel;
                justMovedToInitArm = false;
                armWristInited = true;

            }

            if (gamepad2.right_bumper && gamepad2_rightBumperReleased && armWristInited && (slidesLeft.getCurrentPosition() < 50 || slidesRight.getCurrentPosition() < 50)) {
                armServo.setPosition(armPickupPos);
                leftWrist.setPosition(wristPickupPos);
                rightWrist.setPosition(wristPickupPos);
                wristInPickupPosTime = System.currentTimeMillis();
                gamepad2_rightBumperReleased = false;
                armWristInited = false;
                wristInPickupPos = true;
            }
            if (wristInPickupPos && System.currentTimeMillis() - wristInPickupPosTime >= 400) {
                clawServo.setPosition(clawClosePos);
                clawClosedTime = System.currentTimeMillis();
                wristInPickupPos = false;
                clawJustClosedOnSample = true;
                clawClosed = true;
            }
            if (clawJustClosedOnSample && System.currentTimeMillis() - clawClosedTime >= 150) {
                leftWrist.setPosition(wristVerticalPos);
                rightWrist.setPosition(wristVerticalPos);
                armServo.setPosition(armVerticalPos);
                clawJustClosedOnSample = false;

            }


            if (gamepad2.x && gamepad2_xReleased && clawClosed) {
                targets = midLevel;
                slidesGoingToMid = true;
                gamepad2_xReleased = false;
                atDropPos = true;
            }
            else if (gamepad2.x && gamepad2_xReleased && !clawClosed) {
                leftWrist.setPosition(wristInitPos);
                rightWrist.setPosition(wristInitPos);
                armServo.setPosition(armInitPos);
                targets = midLevel;
                gamepad2_xReleased = false;
                atDropPos = true;
            }

            if (gamepad2.y && gamepad2_yReleased && clawClosed) {
                targets = highLevel;
                slidesGoingtoHigh = true;
                gamepad2_yReleased = false;
                atDropPos = true;
            }
            else if (gamepad2.y && gamepad2_yReleased && !clawClosed) {
                leftWrist.setPosition(wristInitPos);
                rightWrist.setPosition(wristInitPos);
                armServo.setPosition(armInitPos);
                targets = highLevel;
                gamepad2_yReleased = false;
                atDropPos = true;
            }

            if ((slidesGoingToMid || slidesGoingtoHigh) && (slidesLeft.getCurrentPosition() >= targets - startMovingArmBackDistFromTarget || slidesRight.getCurrentPosition() >= targets - startMovingArmBackDistFromTarget)) {
                leftWrist.setPosition(wristDropPos);
                rightWrist.setPosition(wristDropPos);
                armServo.setPosition(armDropPos);
                slidesGoingToMid = false;
                slidesGoingtoHigh = false;
                atDropPos = true;
            }
            if ((gamepad2.left_trigger != 0) && gamepad2_leftTriggerReleased) {
                clawServo.setPosition(clawOpenPos);
                clawClosed = false;
                gamepad2_leftTriggerReleased = false;
            }

            if (gamepad2.right_trigger != 0 && gamepad2_rightTriggerReleased){
                clawServo.setPosition(clawClosePos);
                clawClosed = true;
                gamepad2_rightTriggerReleased = false;
            }
            if (gamepad2.dpad_left && gamepad2_dPadLeftReleased && !atDropPos) {
                double currWristPos = leftWrist.getPosition();
                leftWrist.setPosition(currWristPos - 0.05);
                rightWrist.setPosition(currWristPos - 0.05);
                if (!atDropPos) {
                    wristPickupPos = currWristPos - 0.05;
                }
                gamepad2_dPadLeftReleased = false;
            }
            if (gamepad2.dpad_left && gamepad2_dPadLeftReleased && atDropPos) {
                double currWristPos = leftWrist.getPosition();
                leftWrist.setPosition(currWristPos - 0.05);
                rightWrist.setPosition(currWristPos - 0.05);
                if (atDropPos) {
                    wristDropPos = currWristPos - 0.05;
                }
                gamepad2_dPadLeftReleased = false;
            }
            if (gamepad2.dpad_right && gamepad2_dPadRightReleased && !atDropPos) {
                double currWristPos = leftWrist.getPosition();
                leftWrist.setPosition(currWristPos + 0.05);
                rightWrist.setPosition(currWristPos + 0.05);
                if (!atDropPos) {
                    wristPickupPos = currWristPos + 0.05;
                }
                gamepad2_dPadRightReleased = false;
            }
            if (gamepad2.dpad_right && gamepad2_dPadRightReleased && atDropPos) {
                double currWristPos = leftWrist.getPosition();
                leftWrist.setPosition(currWristPos + 0.05);
                rightWrist.setPosition(currWristPos + 0.05);
                if (atDropPos) {
                    wristDropPos = currWristPos + 0.05;
                }
                gamepad2_dPadRightReleased = false;
            }
            if (gamepad2.dpad_up && gamepad2_dPadUpReleased && !atDropPos) {
                double currArmPos = armServo.getPosition();
                armServo.setPosition(currArmPos - 0.05);
                if (!atDropPos) {
                    armPickupPos = currArmPos - 0.05;
                }
                gamepad2_dPadUpReleased = false;
            }
            if (gamepad2.dpad_up && gamepad2_dPadUpReleased && atDropPos) {
                double currArmPos = armServo.getPosition();
                armServo.setPosition(currArmPos - 0.05);
                if (atDropPos) {
                    armDropPos = currArmPos - 0.05;
                }
                gamepad2_dPadUpReleased = false;
            }
            if (gamepad2.dpad_down && gamepad2_dPadDownReleased && !atDropPos) {
                double currArmPos = armServo.getPosition();
                armServo.setPosition(currArmPos + 0.05);
                if (!atDropPos) {
                    armPickupPos = currArmPos + 0.05;
                }
                gamepad2_dPadDownReleased = false;
            }
            if (gamepad2.dpad_down && gamepad2_dPadDownReleased && atDropPos) {
                double currArmPos = armServo.getPosition();
                armServo.setPosition(currArmPos + 0.05);
                if (atDropPos) {
                    armDropPos = currArmPos + 0.05;
                }
                gamepad2_dPadDownReleased = false;
            }
            //if (gamepad2.left_stick_x >= 0) {
                //
            // targets += 100;

            //}




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
            if (!gamepad1.dpad_right) {
                gamepad1_dPadRightReleased = true;
            }
            if (!gamepad1.dpad_left) {
                gamepad1_dPadLeftReleased = true;
            }

            //ARM MOTOR POSITION TESTING

            if (gamepad1.right_trigger == 0) {
                gamepad1_rightTriggerReleased = true;
            }

            if (!gamepad2.x) {
                gamepad2_xReleased = true;
            }
            if (!gamepad2.y) {
                gamepad2_yReleased = true;
            }
            if (!gamepad2.left_bumper) {
                gamepad2_leftBumperReleased = true;
            }
            if (!gamepad2.right_bumper) {
                gamepad2_rightBumperReleased = true;
            }
            if (gamepad2.right_trigger == 0) {
                gamepad2_rightTriggerReleased = true;
            }
            if (gamepad2.left_trigger == 0) {
                gamepad2_leftTriggerReleased = true;
            }
            if (!gamepad2.dpad_left) {
                gamepad2_dPadLeftReleased = true;
            }
            if (!gamepad2.dpad_right) {
                gamepad2_dPadRightReleased = true;
            }
            if(!gamepad2.dpad_up) {
                gamepad2_dPadUpReleased = true;
            }
            if(!gamepad2.dpad_down) {
                gamepad2_dPadDownReleased = true;
            }


        }
    }

}
