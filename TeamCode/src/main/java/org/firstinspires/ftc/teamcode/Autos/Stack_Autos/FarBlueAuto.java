package org.firstinspires.ftc.teamcode.Autos.Stack_Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.Camera;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Outake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;



@Autonomous(name = "FarBlueAuto")

public class FarBlueAuto extends LinearOpMode {

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

        propDetection = new Camera.PropDetection(hardwareMap,telemetry,true);
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
        Pose2d blueStart = new Pose2d(-40, 61, Math.toRadians(270));
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
                .splineToSplineHeading(new Pose2d(-30, 34, Math.toRadians(315)), Math.toRadians(315))
                .back(4)
                .lineToSplineHeading(new Pose2d(-57.5, 34.5, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    //run intake
                    intake.setToIntake(0.0);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    intake.setToIntake(0.06);
                })
                .waitSeconds(0.5)

                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-35,58.5),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    intake.setToGround();
                })
                .lineToSplineHeading(new Pose2d(25,58.5,Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(45,40),Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    //extend outtake
                     outake.extendOutake(1400);
                })
                .waitSeconds(0)
                .forward(9)
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    //open claw
                     outake.openClaw();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                     outake.resetOutake();
                    //retract outtake
                })

                //
                //2+1
                //

                //GOING TO INTAKE +3
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(30,56.5),Math.toRadians(180))
                .back(2)
                .lineToSplineHeading(new Pose2d(-20,54.5,Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(-54,34.5),Math.toRadians(180))
                .lineToSplineHeading(new Pose2d(-57.0,34.5,Math.toRadians(0)))
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
                .splineToConstantHeading(new Vector2d(-35,55.5),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    intake.setToGround();
                })
                .lineToSplineHeading(new Pose2d(25,55.5,Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(45,40),Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    //extend outtake
                     outake.extendOutake(1400);
                })
                .waitSeconds(0)
                .forward(9)
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    //open claw
                     outake.openClaw();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                     outake.resetOutake();
                    //retract outtake
                })
                //2+3

                .build();



        TrajectorySequence secondBlueTraj = drive.trajectorySequenceBuilder(blueStart)
                .lineToLinearHeading(new Pose2d(-36,32,Math.toRadians(270)))
                .back(4)
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(-58.5,35,Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    //run intake
                    intake.setToIntake(0.0);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    intake.setToIntake(0.06);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{
                    intake.setToIntake(0.05);
                })
                .waitSeconds(0.5)

                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-35,58.5),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    intake.setToGround();
                })
                .lineToSplineHeading(new Pose2d(25,58.5,Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(45,35),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    //extend outtake
                    // outake.extendOutake(1400);
                })
                .waitSeconds(0)
                .forward(9)
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    //open claw
                    // outake.openClaw();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    // outake.resetOutake();
                    //retract outtake
                })
                //
                //2+1
                //

                //GOING TO INTAKE +3
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(30,56.5),Math.toRadians(180))
                .back(2)
                .lineToSplineHeading(new Pose2d(-20,54.5,Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(-54,34.5),Math.toRadians(180))
                .lineToSplineHeading(new Pose2d(-59.0,34.5,Math.toRadians(0)))
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
              //  .UNSTABLE_addTemporalMarkerOffset(1,()->{
                //    intake.setToIntake(0.05);
               // })
                .waitSeconds(1.5)


                //GOING TO DEPOSIT +3
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-35,52.5),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    intake.setToGround();
                })
                .lineToSplineHeading(new Pose2d(25,52.5,Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(45,40),Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    //extend outtake
                    // outake.extendOutake(1400);
                })
                .waitSeconds(0)
                .forward(9)
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    //open claw
                    // outake.openClaw();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    // outake.resetOutake();
                    //retract outtake
                })
                //2+3


                /*

                .setReversed(true)
                .lineToLinearHeading(new Pose2d(40, 25, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(20,12), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(0)))
                .waitSeconds(1)

                .setReversed(false)
                .lineToLinearHeading(new Pose2d(23, 12, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(42,27), Math.toRadians(45))
                .waitSeconds(1)

                */

                .build();


        TrajectorySequence thirdBlueTraj = drive.trajectorySequenceBuilder(blueStart)
                .lineToLinearHeading(new Pose2d(-46,33,Math.toRadians(270)))
                .back(4)
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(80), 11.5) , SampleMecanumDrive.getAccelerationConstraint(45))
                .lineToSplineHeading(new Pose2d(-57.0,35,Math.toRadians(0)))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    //run intake
                    intake.setToIntake(0.0);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    intake.setToIntake(0.06);
                })
                .waitSeconds(1)
                .setReversed(false)
                .resetConstraints()
                .splineToConstantHeading(new Vector2d(-35,58.5),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    intake.setToGround();
                })
                .lineToSplineHeading(new Pose2d(25,58.5,Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(45,30),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    //extend outtake
                    // outake.extendOutake(1400);
                })
                .waitSeconds(0)
                .forward(9)
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    //open claw
                    // outake.openClaw();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    // outake.resetOutake();
                    //retract outtake
                })
                //
                //2+1
                //

                //GOING TO INTAKE +3
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(30,56.5),Math.toRadians(180))
                .back(2)
                .lineToSplineHeading(new Pose2d(-20,54.5,Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(-54,33.5),Math.toRadians(180))
                .lineToSplineHeading(new Pose2d(-57.5,33.5,Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    //run intake
                    intake.setToIntake(0.0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    intake.setToIntake(0.09);
                })
                .UNSTABLE_addTemporalMarkerOffset(1,()->{
                    intake.setToIntake(0.12);
                })
                .waitSeconds(2)


                //GOING TO DEPOSIT +3
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-35,54.5),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    intake.setToGround();
                })
                .lineToSplineHeading(new Pose2d(25,54.5,Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(45,40),Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    //extend outtake
                    // outake.extendOutake(1400);
                })
                .waitSeconds(0)
                .forward(9)
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    //open claw
                    // outake.openClaw();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    // outake.resetOutake();
                    //retract outtake
                })
                //2+3





                /*
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(40, 25, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(20,12), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(0)))
                .waitSeconds(1)

                .setReversed(false)
                .lineToLinearHeading(new Pose2d(23, 12, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(42,27), Math.toRadians(45))
                .waitSeconds(1)

                 */
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
                    propPos = 2;
                    break;
                case BOT_RIGHT:
                    propPos = 3;
                    break;
                case BOT_OTHER:
                    propPos = 1;
                    break;
            }
            telemetry.addData("proppos",propPos);
            telemetry.update();
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
           // outake.executeAuto();
            intake.executeAuto();
            outake.executeAuto();
            drive.update();

        }
    }
}