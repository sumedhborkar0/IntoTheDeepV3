package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class wrist {
    Servo wrist;
    LinearOpMode op;
    public wrist(LinearOpMode opMode){
        op = opMode;
        wrist = op.hardwareMap.servo.get("wrist");

    }
    public void flipUp(){
        wrist.setPosition(0.025);
    }
    public void flipDown(){
        wrist.setPosition(0.7+0.025);

    }
}
