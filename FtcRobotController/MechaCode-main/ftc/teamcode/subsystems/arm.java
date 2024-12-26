package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class arm {
    LinearOpMode op;
    Servo armright;
    Servo armleft;
    public arm(LinearOpMode opMode){
        op = opMode;
        armright = op.hardwareMap.servo.get("rightArm");
        armleft = op.hardwareMap.servo.get("leftArm");

    }
    public void pickUpPos(){
        armright.setPosition(0);
        armleft.setPosition(1);
    }
    public void initPos(){
        armright.setPosition(0.75);
        armleft.setPosition(0.25);
    }
    public void dropPos(){
        armright.setPosition(0.8);
        armleft.setPosition(0.2);
    }
    public void droppedPos(){
        armright.setPosition(1);
        armleft.setPosition(0);
    }
}
