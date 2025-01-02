package org.firstinspires.ftc.teamcode.Teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
        DcMotor outtakeRight = hardwareMap.dcMotor.get("outtakeRight");
        DcMotor outtakeLeft = hardwareMap.dcMotor.get("outtakeLeft");
        Servo rightServo = hardwareMap.servo.get("rightServo");
        Servo leftServo = hardwareMap.servo.get("leftServo");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        outtakeLeft.setDirection(DcMotor.Direction.REVERSE);
        rightServo.setDirection(Servo.Direction.REVERSE);


        leftServo.setPosition(0.5);
        rightServo.setPosition(0.5);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
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
            if (gamepad1.right_trigger != 0) {
                outtakeLeft.setPower(gamepad1.right_trigger);
                outtakeRight.setPower(gamepad1.right_trigger);
            }
            else if (gamepad1.left_trigger != 0){
                outtakeLeft.setPower(-gamepad1.left_trigger);
                outtakeRight.setPower(-gamepad1.left_trigger);
            }
            else {
                outtakeLeft.setPower(0);
                outtakeRight.setPower(0);
            }


            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);

            if (gamepad1.dpad_up) {
                rightServo.setPosition(0.7);
                leftServo.setPosition(0.7);}
            if (gamepad1.dpad_down) {
                rightServo.setPosition(0.5);
                leftServo.setPosition(0.5);
            }



        }
    }
}