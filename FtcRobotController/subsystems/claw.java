package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class claw {
    Servo claw;
    LinearOpMode op;
    public claw(LinearOpMode opMode){
        op = opMode;
        claw = op.hardwareMap.servo.get("claw");


    }
    public void open(){
        claw.setPosition(0.8);
    }
    public void close(){
        claw.setPosition(1);
    }
}
