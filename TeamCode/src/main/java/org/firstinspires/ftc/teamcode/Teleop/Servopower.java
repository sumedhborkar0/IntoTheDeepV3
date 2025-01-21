package org.firstinspires.ftc.teamcode.Teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Servopower extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo1 = hardwareMap.servo.get("servo1");
        Servo servo2 = hardwareMap.servo.get("servo2");
        Servo servo3 = hardwareMap.servo.get("servo3");
        Servo servo4 = hardwareMap.servo.get("servo4");
        Servo servo5 = hardwareMap.servo.get("servo5");
        Servo servo6 = hardwareMap.servo.get("servo6");
        servo1.setPosition(.5);
        while (opModeIsActive()) {
            if (gamepad1.a) {
                servo1.setPosition(.1);

            }
        }

    }
}
