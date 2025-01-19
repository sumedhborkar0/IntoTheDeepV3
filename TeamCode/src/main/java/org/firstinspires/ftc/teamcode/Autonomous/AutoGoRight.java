package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

//WORKS
@Autonomous(name = "AutoGoRight", preselectTeleOp = "TeleopV2")
public class AutoGoRight extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.

        /*// Delcare Trajectory as such
        Action TrajectoryAction1 = drive.actionBuilder(new Pose2d(-37.31, -63.49, Math.toRadians(90.00)))
                .strafeTo(new Vector2d(-55.47, -55.31))
                .strafeTo(new Vector2d(-48.93, -34.04))
                .strafeToLinearHeading(new Vector2d(-58.58, -58.91), Math.toRadians(237.50))
                .strafeToLinearHeading(new Vector2d(-58.75, -34.53), Math.toRadians(94.83))
                .strafeTo(new Vector2d(-58.09, -58.58))
                .strafeToLinearHeading(new Vector2d(-64.15, -35.51), Math.toRadians(90.00))
                .strafeTo(new Vector2d(-61.36, -54.98))
                .build();
                .lineToX(50)
                .lineToX(0)
                .build(); */

        waitForStart();

        if (isStopRequested()) return;
/*
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-60, 0, Math.toRadians(90.00)))
                .lineToY(-45)
                .lineToX(-35)
                .lineToY(-60)
                .lineToY(-45)
                .lineToX(-60)
                .lineToY(0)
                .build());
*/

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))

                        .strafeTo(new Vector2d(2.5, -100))
                        .build());



    }

}

