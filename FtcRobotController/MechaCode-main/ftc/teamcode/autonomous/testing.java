package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous
@Disabled
public class testing extends LinearOpMode {
    DcMotor intake;
    public void runOpMode(){
        intake = hardwareMap.dcMotor.get("intake");
        waitForStart();
        while(opModeIsActive()){
            intake.setPower(-0.5);
        }


    }
}
