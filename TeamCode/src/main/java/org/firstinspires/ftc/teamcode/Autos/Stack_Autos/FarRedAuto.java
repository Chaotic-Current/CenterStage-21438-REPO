package org.firstinspires.ftc.teamcode.Autos.Stack_Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.Camera;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Outake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name = "FarRedAuto")

public class FarRedAuto extends LinearOpMode {

    enum States {
        START,
        IDLE
    }

    States currentState = States.START;
    SampleMecanumDrive drive;

    Camera.PropDetection propDetection;
    Outake outake;
    Intake intake;


    public void initialize() {

        propDetection = new Camera.PropDetection(hardwareMap,telemetry,false);
        propDetection.runPipeline();
       // outake = new Outake(hardwareMap, telemetry);

        outake = new Outake(hardwareMap,telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap,telemetry,outake);

    }

    @Override
    public void runOpMode() {
        int propPos = 0;
        initialize();
        Pose2d redStart = new Pose2d(-40, -60, Math.toRadians(90));
        Pose2d blueCam = new Pose2d(20, 61, Math.toRadians(90));

        Pose2d thirdBlueStripe = new Pose2d(-21, 35, Math.toRadians(270));
        Pose2d secondBlueStripe = new Pose2d(-13, 28, Math.toRadians(270));
        Pose2d firstBlueStripe = new Pose2d(-3, 34, Math.toRadians(315));

        Pose2d thirdBlueScoreArea = new Pose2d(45, 29, Math.toRadians(0));
        Pose2d secondBlueScoreArea = new Pose2d(50, 32, Math.toRadians(0));
        Pose2d firstBlueScoreArea = new Pose2d(50, 41, Math.toRadians(0));


        drive.setPoseEstimate(redStart);

        //if you don't know when to relocalize ima disown you
        TrajectorySequence firstBlueTraj = drive.trajectorySequenceBuilder(redStart)
                //.waitSeconds(1.5)
                .lineToLinearHeading(new Pose2d(-46,-33,Math.toRadians(90)))
                .back(4)
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(50), 11.5) , SampleMecanumDrive.getAccelerationConstraint(40))
                .lineToSplineHeading(new Pose2d(-56.0,-34,Math.toRadians(0)))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    //run intake
                    intake.setToIntake(0.0);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    intake.setToIntake(0.058);
                })
                .waitSeconds(0.5)

                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-35,-59),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    intake.twerk();
                })
                .lineToSplineHeading(new Pose2d(25,-59,Math.toRadians(-2)))
                .splineToConstantHeading(new Vector2d(45,-30),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    intake.setToGround();
                })
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    //extend outtake
                    outake.extendOutake(1400);
                })
                .waitSeconds(0)
                .forward(9)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //open claw
                    outake.halfOpenClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    //open claw
                    outake.openClaw();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    outake.resetOutake();
                    //retract outtake
                })

                //
                //2+1
                //

                //GOING TO INTAKE +3
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(30,-58),Math.toRadians(180))
                .back(2)
                .lineToSplineHeading(new Pose2d(-25,-58,Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(-54,-34),Math.toRadians(180))
                .lineToSplineHeading(new Pose2d(-57.0,-34,Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    //run intake
                    intake.setToIntake(0.0);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    intake.setToIntake(0.09);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{
                    intake.setToIntake(0.12);
                })
                .waitSeconds(1.5)


                //GOING TO DEPOSIT +3
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-35,-59),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                   // intake.setToGround();
                    intake.twerk();
                })
                .lineToSplineHeading(new Pose2d(25,-58.5,Math.toRadians(-2)))
                .splineToConstantHeading(new Vector2d(45,-42),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    // intake.setToGround();
                    intake.setToGround();
                })

                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    //extend outtake
                    outake.extendOutake(1400);
                })
                .waitSeconds(0)
                .forward(9)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //open claw
                    outake.halfOpenClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    //open claw
                    outake.openClaw();
                })
                .waitSeconds(0.5)
                .back(7)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    outake.resetOutake();
                    //retract outtake
                })
                //2+3

                .build();



        TrajectorySequence secondBlueTraj = drive.trajectorySequenceBuilder(redStart)
               // .waitSeconds(1.5)
                .lineToLinearHeading(new Pose2d(-36,-32,Math.toRadians(90)))
                .back(4)
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(-57,-35,Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    //run intake
                    intake.setToIntake(0.0);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.8,()->{
                    intake.setToIntake(0.055);
                })
                .waitSeconds(0.5)

                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-35,-59),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                   // intake.setToGround();
                    intake.twerk();
                })
                .lineToLinearHeading(new Pose2d(25,-58.5,Math.toRadians(-2)))
                .lineToLinearHeading(new Pose2d(45,-40,Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    intake.setToGround();
                })

                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    //extend outtake
                    outake.extendOutake(1400);
                })
                .waitSeconds(0)
                .forward(9)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //open claw
                    outake.halfOpenClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    //open claw
                    outake.openClaw();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    outake.resetOutake();
                    //retract outtake
                })

                //
                //2+1
                //

                //GOING TO INTAKE +3
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(30,-58),Math.toRadians(180))
                .back(2)
                .lineToSplineHeading(new Pose2d(-25,-58,Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(-54,-34),Math.toRadians(180))
                .lineToSplineHeading(new Pose2d(-57.0,-34,Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    //run intake
                    intake.setToIntake(0.0);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    intake.setToIntake(0.09);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{
                    intake.setToIntake(0.12);
                })
                .waitSeconds(1.5)


                //GOING TO DEPOSIT +3
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-35,-59),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    intake.twerk();
                })
                .lineToLinearHeading(new Pose2d(25,-58.5,Math.toRadians(-2)))
                .lineToLinearHeading(new Pose2d(45,-45,Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    intake.twerk();
                })
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    //extend outtake
                    outake.extendOutake(1400);
                })
                .waitSeconds(0)
                .forward(9)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //open claw
                    outake.halfOpenClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    //open claw
                    outake.openClaw();
                })
                .waitSeconds(0.5)
                .back(7)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    outake.resetOutake();
                    //retract outtake
                })
                //2+3

                .build();


        TrajectorySequence thirdBlueTraj = drive.trajectorySequenceBuilder(redStart)
              //  .waitSeconds(1.5)
                .splineToSplineHeading(new Pose2d(-30, -34, Math.toRadians(45)), Math.toRadians(45))
                .back(4)
                .lineToSplineHeading(new Pose2d(-57, -34.5, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    //run intake
                    intake.setToIntake(0.0);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    intake.setToIntake(0.06);
                })
                .waitSeconds(0.5)

                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-35,-59),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    intake.twerk();
                })
                .lineToLinearHeading(new Pose2d(25,-59,Math.toRadians(-2)))
                .lineToLinearHeading(new Pose2d(45,-40,Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    intake.setToGround();
                })
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    //extend outtake
                    outake.extendOutake(1400);
                })
              //  .waitSeconds(0)
                .forward(9)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //open claw
                    outake.halfOpenClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    //open claw
                    outake.openClaw();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    outake.resetOutake();
                    //retract outtake
                })

                //
                //2+1
                //

                //GOING TO INTAKE +3
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(30,-58),Math.toRadians(180))
                .back(2)
                .lineToSplineHeading(new Pose2d(-25,-58,Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(-54,-34.5),Math.toRadians(180))
                .lineToSplineHeading(new Pose2d(-57.0,-34.5,Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    //run intake
                    intake.setToIntake(0.0);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    intake.setToIntake(0.09);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{
                    intake.setToIntake(0.12);
                })
                .waitSeconds(1.5)


                //GOING TO DEPOSIT +3
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-35,-59),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    intake.twerk();
                })
                .lineToLinearHeading(new Pose2d(25,-58.5,Math.toRadians(-2)))
                .lineToLinearHeading(new Pose2d(45,-47, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    intake.setToGround();
                })
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    //extend outtake
                    outake.extendOutake(1400);
                })
                //.waitSeconds(0)
                .forward(9)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //open claw
                    outake.halfOpenClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    //open claw
                    outake.openClaw();
                })
                .waitSeconds(0.5)
                .back(7)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    outake.resetOutake();
                    //retract outtake
                })
                //2+3

                .build();

        //waitForStart();
        //running pipeline after start button is pressed, its a bet, but have to experiement and see if calling runpipline once
        // before the command waitForStart(); causes the camera to be running the entire auto, or stops after init
        //cam.runPipeline();

        //drive.followTrajectorySequenceAsync(camDetection);
        currentState = States.START;

        while (opModeInInit()) {
            switch(propDetection.getPosition()){
                case BOT_LEFT:
                    propPos = 1;
                    break;
                case BOT_RIGHT:
                    propPos = 2;
                    break;
                case BOT_OTHER:
                    propPos = 3;
                    break;
            }
            telemetry.addData("proppos",propPos);
            telemetry.update();
        }

        while (opModeIsActive()) {
          //  propPos = 1;
            switch (currentState) {
                case START:
                    if (!drive.isBusy()) {



                        switch (propPos) {
                            case 1:
                                drive.followTrajectorySequenceAsync(firstBlueTraj);
                                currentState = FarRedAuto.States.IDLE;
                                break;
                            case 2:
                                drive.followTrajectorySequenceAsync(secondBlueTraj);
                                currentState = FarRedAuto.States.IDLE;
                                break;
                            case 3:
                                drive.followTrajectorySequenceAsync(thirdBlueTraj);
                                currentState = FarRedAuto.States.IDLE;
                                break;
                        }


                    }
                    break;
                case IDLE:
                    break;
                    
            }
           // outake.executeAuto();
            intake.executeAuto();
            outake.executeAuto();
            drive.update();

        }
    }
}