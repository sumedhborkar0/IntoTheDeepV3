package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.computervision.CVMaster;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@TeleOp
public class teleop extends LinearOpMode {
    public Servo microLa;
    public DcMotor intake;
    public Servo servoleft;
    public Servo servoright;
    public DcMotor leftFront;
    public DcMotor leftRear;
    public DcMotor rightFront;
    public DcMotor rightRear;
    public DcMotor slidesleft;
    public DcMotor slidesright;
    public Servo micro;
    public Servo launcher;
    private PIDController slidescontroller;
    public static double ps = 0.002, is = 0, ds = 0;
    public static double fs = 0.01;
    public int targets = 0;
    private final double ticks_in_degrees = 384 / 360.0;
    public boolean pressed = false;

    public int power = 0;
    public enum Drop{
        state1,
        state2,
        state3 }




    public void runOpMode() {
        intake = hardwareMap.dcMotor.get("intake");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        slidescontroller = new PIDController(ps, is, ds);
        slidesleft = hardwareMap.dcMotor.get("slidesleft");
        slidesleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        micro = hardwareMap.servo.get("micro");
        servoleft = hardwareMap.servo.get("servoleft");
        servoright = hardwareMap.servo.get("servoright");

        slidesleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        microLa = hardwareMap.servo.get("MicroLauncher");
//slidesleft.setDirection(DcMotor.Direction.REVERSE);


        slidesright = hardwareMap.dcMotor.get("s");
        slidesright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesright.setDirection(DcMotor.Direction.REVERSE);
        launcher = hardwareMap.servo.get("launcher");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        servoleft.setPosition(0.068);
        servoright.setPosition(0.932);
        micro.setPosition(1);
// CVMaster cv = new CVMaster(this);
// cv.observePixel();
        launcher.setPosition(0.4);
        microLa.setPosition(0);
        ElapsedTime time = new ElapsedTime();
        Drop drop = Drop.state3;

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            intake.setPower(power);
            if (gamepad1.a) {
                power = -1;
            } else if (gamepad1.right_bumper) {
                power = 1;
            }
            else if (gamepad1.left_bumper) {
                power = 0;
            }
            if (gamepad2.x) {
                targets = 0;
                servoleft.setPosition(0.02);
                servoright.setPosition(0.98);
                micro.setPosition(1);
            } else if (gamepad2.b) {
                servoleft.setPosition(0.2);
                servoright.setPosition(0.8);
            } else if (gamepad2.left_bumper) {
                micro.setPosition(0.5);
            } else if (gamepad1.right_bumper) {
                launcher.setPosition(0.5);
// micro.setPosition(1);
            }
            if(pressed == false){
                if (gamepad2.right_bumper){
                    drop = Drop.state1;
                    pressed = true;

                }
            }

            if (gamepad1.dpad_left) {
                launcher.setPosition(0);

            }
            else if(gamepad1.dpad_right){
                microLa.setPosition(0);
                sleep(500);
                launcher.setPosition(0.5);
            }
            if (gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0) {
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
            }
            if (gamepad2.dpad_up) {
                targets = 3900;
                servoleft.setPosition(0.5);
                servoright.setPosition(0.5);
            }
            if (gamepad2.dpad_right) {
                targets = 2500;
                servoleft.setPosition(0.5);
                servoright.setPosition(0.5);
            }
            if (gamepad2.dpad_down){
                targets = 1900;
                servoleft.setPosition(0.5);
                servoright.setPosition(0.5);
            }
            if(gamepad2.y){
                targets = targets + 20;
            }
            if(gamepad2.a){
                targets = targets - 20;
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            (-gamepad1.left_stick_y)*0.71,
                            (-gamepad1.left_stick_x)*0.71,
                            (-gamepad1.right_stick_x)*0.71
                    )
            );


            drive.update();
            telemetry.addData("right", slidesright.getCurrentPosition());
            telemetry.addData("left", slidesleft.getCurrentPosition());
            telemetry.update();
            slidescontroller.setPID(ps, is, ds);
            int slidesleftpos = slidesleft.getCurrentPosition();
            double pidleft = slidescontroller.calculate(slidesleftpos, targets);
            double ffleft = Math.cos(Math.toRadians(targets / ticks_in_degrees)) * fs;

            int slidesrightpos = slidesright.getCurrentPosition();
            double pidright = slidescontroller.calculate(slidesrightpos, targets);
            double ffright = Math.cos(Math.toRadians(targets / ticks_in_degrees)) * fs;

            double powerleft = pidleft + ffleft;

            double powerright = pidright + ffright;
            slidesleft.setPower(powerleft);
            slidesright.setPower(powerright);
            switch(drop){
                case state1:
                    micro.setPosition(0.5);
                    time.reset();
                    drop = Drop.state2;

                    break;

                case state2:
                    if(time.seconds()>0.2){
                        micro.setPosition(1);
                        drop = Drop.state3;
                        pressed = false;
                    }
                    break;
                case state3:
                    break;


            }
        }
    }
}