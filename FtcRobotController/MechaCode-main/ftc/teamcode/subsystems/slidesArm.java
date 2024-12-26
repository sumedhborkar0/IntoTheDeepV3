package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class slidesArm {
    LinearOpMode op;
    private PIDController slidescontroller;
    public static double ps = 0.002, is = 0, ds = 0;
    public static double fs = 0.01;
    public int targets = 0;
    private final double ticks_in_degrees = 384 / 360.0;
    public DcMotor slidesLeft;
    public DcMotor slidesRight;

    public slidesArm(LinearOpMode opMode){
        op = opMode;
        slidescontroller = new PIDController(ps, is, ds);
        slidesLeft = op.hardwareMap.dcMotor.get("slidesLeft");
        slidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slidesLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        slidesRight = op.hardwareMap.dcMotor.get("slidesRight");
        slidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    public void updateSlides(){
        slidescontroller.setPID(ps, is, ds);
        int slidesleftpos = slidesLeft.getCurrentPosition();
        double pidleft = slidescontroller.calculate(slidesleftpos, targets);
        double ffleft = Math.cos(Math.toRadians(targets / ticks_in_degrees)) * fs;

        int slidesrightpos = slidesRight.getCurrentPosition();
        double pidright = slidescontroller.calculate(slidesrightpos, targets);
        double ffright = Math.cos(Math.toRadians(targets / ticks_in_degrees)) * fs;

        double powerleft = pidleft + ffleft;

        double powerright = pidright + ffright;

        slidesLeft.setPower(powerleft);
        slidesRight.setPower(powerright);
    }
}
