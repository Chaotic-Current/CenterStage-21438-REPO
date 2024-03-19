package org.firstinspires.ftc.teamcode.Autos.Stack_Autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.Camera;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Outake;
import org.firstinspires.ftc.teamcode.Pipelines.DetectColor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Mechanisms.MechanismTests.AprilTagCam;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "NearBlueAuto")
@Config

public class NearBlueAuto extends LinearOpMode {

    enum States {
        START,
        IDLE
    }

    States currentState = States.START;
    SampleMecanumDrive drive;

    Camera.PropDetection propDetection;

    private AprilTagCam frontTagCam;
    private OpenCvWebcam cam;

    DetectColor detector;
    Outake outake;
    Intake intake;
    public static double boardX = 44;



    public void initialize() {

        propDetection = new Camera.PropDetection(hardwareMap,telemetry,true);
        propDetection.runPipeline();
      //  frontTagCam = new AprilTagCam(hardwareMap,"WebcamFront",6);
       // frontTagCam.setTelemetry(telemetry);

        outake = new Outake(hardwareMap, telemetry);
        intake = new Intake(hardwareMap,telemetry,outake);
        drive = new SampleMecanumDrive(hardwareMap);
     //   cameraInit();
        outake.closeClaw();
    }


    @Override
    public void runOpMode() {
        int propPos = 1;
        initialize();
        Pose2d blueStart = new Pose2d(19.6, 60.5, Math.toRadians(270));
        Pose2d blueCam = new Pose2d(20, 61, Math.toRadians(90));

        Pose2d thirdBlueStripe = new Pose2d(3, 34, Math.toRadians(225));
        Pose2d secondBlueStripe = new Pose2d(13, 28, Math.toRadians(270));
        Pose2d firstBlueStripe = new Pose2d(21, 35, Math.toRadians(270));

        Pose2d thirdBlueScoreArea = new Pose2d(45, 29, Math.toRadians(0));
        Pose2d secondBlueScoreArea = new Pose2d(50, 32, Math.toRadians(0));
        Pose2d firstBlueScoreArea = new Pose2d(50, 41, Math.toRadians(0));


        drive.setPoseEstimate(blueStart);

        TrajectorySequence firstBlueTraj = drive.trajectorySequenceBuilder(blueStart)
                .lineToLinearHeading(new Pose2d(14, 50, Math.toRadians(270)))
                .splineToSplineHeading(new Pose2d(20, 35, Math.toRadians(315)), Math.toRadians(315))
                .setReversed(true)
                .back(11) //tune this distance frfr
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    outake.extendOutake(500);
                    //extend outtake
                })
                .lineToSplineHeading(new Pose2d(33, 43, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(40, 40), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    //extend outtake
                    outake.extendOutake(1200);
                })
                .waitSeconds(0)
                .forward(17)
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    //open claw
                    outake.openClaw();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    outake.resetOutake();
                    //retract outtake
                })


                //relocalize



                .lineToLinearHeading(new Pose2d(40, 15, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(30,10.5), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-55.3, 10.5, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    //run intake
                    intake.setToIntake(0.0);
                })
                .UNSTABLE_addTemporalMarkerOffset(-1.5,()->{
                    intake.setToIntake(0.06);
                })
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    intake.setToIntake(0.08);
                })
                /*
                .waitSeconds(1)
                .setReversed(false)
                .forward(5)
                .back(5)
                .UNSTABLE_addTemporalMarkerOffset(-1.5,()->{
                    //intake.setMotorPow(0.63);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{

                    intake.setToIntake(0.08);
                })

                 */
                .waitSeconds(2)
               // .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(30, 12, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    intake.setToGround();
                })
                .splineToConstantHeading(new Vector2d(42,32), Math.toRadians(45))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    //extend outtake
                    outake.extendOutake(1300);
                })
                .waitSeconds(0)
                .forward(16.0)
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    //open claw
                    outake.halfOpenClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //open claw
                    outake.openClaw();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    outake.resetOutake();
                    //retract outtake
                })


                .lineToLinearHeading(new Pose2d(47,16,Math.toRadians(0)))


                .build();



        TrajectorySequence secondBlueTraj = drive.trajectorySequenceBuilder(blueStart)
                .lineToLinearHeading(new Pose2d(14, 30, Math.toRadians(270)))
                .setReversed(true)

                .back(11) //tune this distance frfr
              //  .waitSeconds(0.25)
                .lineToSplineHeading(new Pose2d(36, 40, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(boardX, 32), Math.toRadians(0))
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


                //relocalize



                .lineToLinearHeading(new Pose2d(40, 15, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(30,12), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-55.5, 12, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    //run intake
                    intake.setToIntake(0.0);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    intake.setToIntake(0.05);
                })
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    intake.setToIntake(0.09);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{
                    intake.setToIntake(0.02);
                })
                .waitSeconds(1)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(30, 12, Math.toRadians(0)))

                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    intake.setToGround();
                })
                .splineToConstantHeading(new Vector2d(42,32), Math.toRadians(45))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    //extend outtake
                    // outake.extendOutake(1300);
                })
                .waitSeconds(0)
                .forward(11.0)
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    //open claw
                    //  outake.halfOpenClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //open claw
                    // outake.openClaw();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    // outake.resetOutake();
                    //retract outtake
                })


                .lineToLinearHeading(new Pose2d(47,16,Math.toRadians(0)))







                .build();


        TrajectorySequence thirdBlueTraj = drive.trajectorySequenceBuilder(blueStart)
                .lineToLinearHeading(new Pose2d(14, 40, Math.toRadians(270)))
                .splineToSplineHeading(new Pose2d(10, 33, Math.toRadians(210)), Math.toRadians(225))
                .setReversed(true)
                //.back(10) //tune this distance frfr
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //extend outtake
                })
                //.splineToConstantHeading(new Vector2d(19, 40), Math.toRadians(0))
                //.lineToSplineHeading(new Pose2d(36, 40, Math.toRadians(0)))
                //.splineToConstantHeading(new Vector2d(boardX, 31), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(boardX,30,Math.toRadians(0)))
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


                //relocalize



                .lineToLinearHeading(new Pose2d(40, 15, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(30,12), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-55.0, 12, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    //run intake
                    intake.setToIntake(0.0);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    intake.setToIntake(0.05);
                })
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    intake.setToIntake(0.09);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{
                    intake.setToIntake(0.02);
                })
                .waitSeconds(1)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(30, 12, Math.toRadians(0)))

                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    intake.setToGround();
                })
                .splineToConstantHeading(new Vector2d(42,32), Math.toRadians(45))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    //extend outtake
                    // outake.extendOutake(1300);
                })
                .waitSeconds(0)
                .forward(11.0)
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    //open claw
                    //  outake.halfOpenClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //open claw
                    // outake.openClaw();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    // outake.resetOutake();
                    //retract outtake
                })


                .lineToLinearHeading(new Pose2d(47,16,Math.toRadians(0)))



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
           // propPos = PropDetection.getPosition();
            /*
            switch(detector.getLocate()){
                case LEFT:
                    propPos = 1;
                    break;
                case CENTER:
                    propPos = 2;
                    break;
                case RIGHT:
                    propPos = 3;
                    break;
            }
            */
            telemetry.addData("proppos",propPos);
            telemetry.update();

            propPos = 1;
            //currentDetectionState = cam.getDetectionState();
        }

        while (opModeIsActive()) {
            propPos = 1;
            switch (currentState) {
                case START:
                    if (!drive.isBusy()) {
                        switch (propPos) {
                            case 1:
                                drive.followTrajectorySequenceAsync(firstBlueTraj);
                                currentState = NearBlueAuto.States.IDLE;
                                break;
                            case 2:
                                drive.followTrajectorySequenceAsync(secondBlueTraj);
                                currentState = NearBlueAuto.States.IDLE;
                                break;
                            case 3:
                                drive.followTrajectorySequenceAsync(thirdBlueTraj);
                                currentState = NearBlueAuto.States.IDLE;
                                break;
                        }


                    }
                    break;
                case IDLE:
                    break;
                    
            }
            outake.executeAuto();
            intake.executeAuto();
            drive.update();
            //frontTagCam.aprilTagUpdate();

        }
    }
}