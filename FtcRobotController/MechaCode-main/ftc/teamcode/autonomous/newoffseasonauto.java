package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.arm;
import org.firstinspires.ftc.teamcode.subsystems.claw;
import org.firstinspires.ftc.teamcode.subsystems.slidesArm;
import org.firstinspires.ftc.teamcode.subsystems.wrist;

@Autonomous

@Disabled

public class newoffseasonauto extends LinearOpMode {
    SampleMecanumDrive drive;
    claw claw; // defining the claw class
    wrist wrist;// defining the wrist class
    arm arm;// defining the arm class
    slidesArm slides;
    public enum Path{
        PRELOAD_TRAJ,
        DROP_CONE,
        HIGH_POLE_TO_SUBSTATION_1,
        PICK_UP_CONE,
        SUBSTATION_TO_POLE1,


    }
    //defining the trajectories

    public enum PickUpCone{

        CLOSE_CLAW,
        LIFT_SLIDES,
        FINISHED,
        DONE
    }

    //defining the states of the robots
    PickUpCone pickUpCone = PickUpCone.CLOSE_CLAW;
    ElapsedTime time = new ElapsedTime();
    double waitTime = 0.5;
    public void runOpMode() {
//initializing the robots
        drive = new SampleMecanumDrive(hardwareMap);
        claw = new claw(this);
        wrist = new wrist(this);
        arm = new arm(this);
        slides = new slidesArm(this);
        claw.close();
        wrist.flipDown();
        arm.initPos();



// defining the trajectories
        TrajectorySequence preloadTrajectory = drive.trajectorySequenceBuilder(new Pose2d(40.89, 64.13, Math.toRadians(90.00)))
                .setReversed(true)
                .addTemporalMarker(() -> {
                    arm.dropPos();
                    slides.targets = 1800;

                })
                .splineTo(new Vector2d(35.04, 45.47), Math.toRadians(270.00))
                .splineTo(new Vector2d(29.5, 12), Math.toRadians(240.00))
                .build();
        TrajectorySequence highPoleToSubstation = drive.trajectorySequenceBuilder(preloadTrajectory.end())
                .splineTo(new Vector2d(57.55, 12.35), Math.toRadians(-3.18))
                .build();



        drive.setPoseEstimate(new Pose2d(40.89, 64.13, Math.toRadians(90.00)));


        drive.followTrajectorySequenceAsync(preloadTrajectory);
        Path path = Path.PRELOAD_TRAJ;
        waitForStart();


        while (opModeIsActive()) {
            switch(path){
                case PRELOAD_TRAJ:
                    if(!drive.isBusy()){
                        path = Path.DROP_CONE;
                        time.reset();

                    }
                    break;
                case DROP_CONE:
                    if(time.seconds()>=waitTime){
                        dropCone();
                        path = Path.HIGH_POLE_TO_SUBSTATION_1;

                    }
                    break;
                case HIGH_POLE_TO_SUBSTATION_1:
                    path = Path.PICK_UP_CONE;
                    drive.followTrajectorySequenceAsync(highPoleToSubstation);
                    break;
                case PICK_UP_CONE:
                    if(!drive.isBusy()){
                        pickUpCone();
                        if(pickUpCone == PickUpCone.DONE){
                            pickUpCone = PickUpCone.CLOSE_CLAW;
                            path = Path.SUBSTATION_TO_POLE1;
                        }


                    }
                    break;
                case SUBSTATION_TO_POLE1:

                    break;
            }


            drive.update();
            slides.updateSlides();
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = poseEstimate;
        }

    }

    //functions to drop and pick up cones
    void dropCone(){
        claw.open();
        arm.droppedPos();
    }
    void pickUpCone(){
        switch (pickUpCone){
            case CLOSE_CLAW:
                claw.close();
                time.reset();
                pickUpCone = PickUpCone.LIFT_SLIDES;

                break;
            case LIFT_SLIDES:
                if(time.seconds()>=waitTime){
                    slides.targets = 600;
                    pickUpCone = PickUpCone.FINISHED;

                }
                break;
            case FINISHED:
                if(slides.slidesLeft.getCurrentPosition()+10>600){
                    pickUpCone = PickUpCone.DONE;
                }

        }


    }
}
