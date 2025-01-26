package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.controller.PIDFController;

public class myPIDAction {
    private DcMotor slidesLeft;
    private DcMotor slidesRight;
    public double targets = 100;

    public myPIDAction(HardwareMap hardwareMap) {
        slidesLeft = hardwareMap.get(DcMotor.class, "slidesLeft");
        slidesRight = hardwareMap.get(DcMotor.class, "slidesRight");

        slidesRight.setDirection(DcMotorEx.Direction.REVERSE);

        slidesLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slidesRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slidesLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slidesRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public class CalcPID implements Action {
        double kp = 0.004, ki = 0, kd = 0, kf = 0.0000007;
        PIDFController controller = new PIDFController(kp, ki, kd, kf);
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            int slidesleftpos = slidesLeft.getCurrentPosition();
            double powerLeft = controller.calculate(slidesleftpos, targets);

            int slidesrightpos = slidesRight.getCurrentPosition();
            double powerRight = controller.calculate(slidesrightpos, targets);

            double powerleft = powerLeft;

            double powerright = powerRight;

            slidesLeft.setPower(powerleft);
            slidesRight.setPower(powerright);
            return true;
        }
    }

    public Action calcPID() {
        return new CalcPID();
    }
}