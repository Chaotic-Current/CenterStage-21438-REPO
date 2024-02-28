package org.firstinspires.ftc.teamcode.Autos.Stack_Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.Camera;
import org.firstinspires.ftc.teamcode.Mechanisms.Outake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;



@Autonomous(name = "FarBlueAuto")
@Disabled
public class FarBlueAuto extends LinearOpMode {

    enum States {
        START,
        IDLE
    }

    States currentState = States.START;
    SampleMecanumDrive drive;

    Camera.PropDetection PropDetection;
    Outake outake;


    public void initialize() {

        //PropDetection = new Camera.PropDetection(hardwareMap,telemetry);
        PropDetection.runPipeline();
        outake = new Outake(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        outake.closeClaw();
    }

    @Override
    public void runOpMode() {
        int propPos = 0;
        initialize();
        Pose2d blueStart = new Pose2d(-35, 61, Math.toRadians(270));
        Pose2d blueCam = new Pose2d(20, 61, Math.toRadians(90));

        Pose2d thirdBlueStripe = new Pose2d(-21, 35, Math.toRadians(270));
        Pose2d secondBlueStripe = new Pose2d(-13, 28, Math.toRadians(270));
        Pose2d firstBlueStripe = new Pose2d(-3, 34, Math.toRadians(315));

        Pose2d thirdBlueScoreArea = new Pose2d(45, 29, Math.toRadians(0));
        Pose2d secondBlueScoreArea = new Pose2d(50, 32, Math.toRadians(0));
        Pose2d firstBlueScoreArea = new Pose2d(50, 41, Math.toRadians(0));


        drive.setPoseEstimate(blueStart);

        //if you don't know when to relocalize ima disown you
        TrajectorySequence firstBlueTraj = drive.trajectorySequenceBuilder(blueStart)
                .lineToLinearHeading(new Pose2d(-35, 40, Math.toRadians(270)))
                //.splineToSplineHeading(some position) for spike IMPORTANT!!!!!
                .setReversed(true)
                .back(10) //tune this frfr
                .lineToLinearHeading(new Pose2d(-35, 55, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(-20, 60), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(19, 60, Math.toRadians(270)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //extend outtake
                })
                .splineToSplineHeading(new Pose2d(45.5, 34, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //open claw
                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //retract outtake
                })
                .lineToLinearHeading(new Pose2d(40, 25, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(20,12), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //run intake
                })
                .waitSeconds(2)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(23, 12, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //extend outtake
                })
                .splineToConstantHeading(new Vector2d(42,27), Math.toRadians(45))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //open claw
                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //retract outtake
                })
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(35, 20, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(20,12), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //run intake
                })
                .waitSeconds(2)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(23, 12, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //extend outtake
                })
                .splineToConstantHeading(new Vector2d(42,27), Math.toRadians(45))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //open claw
                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //reset outtake
                })
                .back(7)
                .build();



        TrajectorySequence secondBlueTraj = drive.trajectorySequenceBuilder(blueStart)
                .lineToLinearHeading(new Pose2d(-35, 32, Math.toRadians(270)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-35, 55, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(-20, 60), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(19, 60, Math.toRadians(270)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //extend outtake
                })
                .splineToSplineHeading(new Pose2d(45.5, 34, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //open claw
                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //retract outtake
                })
                .lineToLinearHeading(new Pose2d(40, 25, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(20,12), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //run intake
                })
                .waitSeconds(2)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(23, 12, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //extend outtake
                })
                .splineToConstantHeading(new Vector2d(42,27), Math.toRadians(45))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //open claw
                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //retract outtake
                })
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(35, 20, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(20,12), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //run intake
                })
                .waitSeconds(2)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(23, 12, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //extend outtake
                })
                .splineToConstantHeading(new Vector2d(42,27), Math.toRadians(45))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //open claw
                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //reset outtake
                })
                .back(7)
                .build();


        TrajectorySequence thirdBlueTraj = drive.trajectorySequenceBuilder(blueStart)
                .lineToLinearHeading(new Pose2d(-35, 40, Math.toRadians(270)))
                //.splineToSplineHeading(some position) for spike IMPORTANT!!!!!
                .setReversed(true)
                .back(10) //tune this frfr
                .lineToLinearHeading(new Pose2d(-35, 55, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(-20, 60), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(19, 60, Math.toRadians(270)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //extend outtake
                })
                .splineToSplineHeading(new Pose2d(45.5, 34, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //open claw
                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //retract outtake
                })
                .lineToLinearHeading(new Pose2d(40, 25, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(20,12), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //run intake
                })
                .waitSeconds(2)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(23, 12, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //extend outtake
                })
                .splineToConstantHeading(new Vector2d(42,27), Math.toRadians(45))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //open claw
                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //retract outtake
                })
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(35, 20, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(20,12), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //run intake
                })
                .waitSeconds(2)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(23, 12, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //extend outtake
                })
                .splineToConstantHeading(new Vector2d(42,27), Math.toRadians(45))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //open claw
                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //reset outtake
                })
                .back(7)
                .build();

        //waitForStart();
        //running pipeline after start button is pressed, its a bet, but have to experiement and see if calling runpipline once
        // before the command waitForStart(); causes the camera to be running the entire auto, or stops after init
        //cam.runPipeline();

        //drive.followTrajectorySequenceAsync(camDetection);
        currentState = States.START;

        while (opModeInInit()) {
            // propPos = PropDetection.getPosition();
            propPos = 2;
            //currentDetectionState = cam.getDetectionState();
        }

        while (opModeIsActive()) {
            switch (currentState) {
                case START:
                    if (!drive.isBusy()) {



                        switch (propPos) {
                            case 1:
                                drive.followTrajectorySequenceAsync(firstBlueTraj);
                                currentState = FarBlueAuto.States.IDLE;
                                break;
                            case 2:
                                drive.followTrajectorySequenceAsync(secondBlueTraj);
                                currentState = FarBlueAuto.States.IDLE;
                                break;
                            case 3:
                                drive.followTrajectorySequenceAsync(thirdBlueTraj);
                                currentState = FarBlueAuto.States.IDLE;
                                break;
                        }


                    }
                    break;
                case IDLE:
                    break;
                    
            }
            outake.executeAuto();
            drive.update();

        }
    }
}