package org.firstinspires.ftc.teamcode.Autos.Stack_Autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.Camera;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Outake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Mechanisms.MechanismTests.AprilTagCam;



@Autonomous(name = "NearBlueAuto")
@Config
public class NearBlueAuto extends LinearOpMode {

    enum States {
        START,
        IDLE
    }

    States currentState = States.START;
    SampleMecanumDrive drive;

    Camera.PropDetection PropDetection;

    private AprilTagCam frontTagCam;
    Outake outake;
    Intake intake;
    public static double boardX = 44;


    public void initialize() {

       // PropDetection = new Camera.PropDetection(hardwareMap,telemetry);
        // PropDetection.runPipeline();
        frontTagCam = new AprilTagCam(hardwareMap,"WebcamFront",6);
        frontTagCam.setTelemetry(telemetry);
        outake = new Outake(hardwareMap, telemetry);
        intake = new Intake(hardwareMap,telemetry,outake);
        drive = new SampleMecanumDrive(hardwareMap);
        //outake.closeClawB();
        //outake.closeClawU();
    }


    @Override
    public void runOpMode() {
        int propPos = 2;
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
                .splineToSplineHeading(new Pose2d(18, 35, Math.toRadians(315)), Math.toRadians(315))
                .setReversed(true)
                .back(20) //tune this distance frfr
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                   // outake.extendOutake(500);
                    //extend outtake
                })
                .lineToSplineHeading(new Pose2d(36, 40, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(60, 31), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //outake.openClawB();
                    //open claw
                })
                .waitSeconds(2)
                //relocalize
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //outake.resetOutake();
                    //retract outtake
                })
                /*
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
                //relocalize
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
                    //retract outtake
                })

                 */
                .back(7)





                .build();



        TrajectorySequence secondBlueTraj = drive.trajectorySequenceBuilder(blueStart)
                .lineToLinearHeading(new Pose2d(14, 30, Math.toRadians(270)))
                .setReversed(true)

                .back(20) //tune this distance frfr
              //  .waitSeconds(0.25)
                .lineToSplineHeading(new Pose2d(36, 40, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(boardX, 32), Math.toRadians(0))
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


                //relocalize



                .lineToLinearHeading(new Pose2d(40, 25, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(20,12), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-54.5, 11, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    //run intake
                    intake.setToIntake(0.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    intake.setToIntake(0.76);
                })
                .waitSeconds(1)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(23, 12, Math.toRadians(0)))

                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    intake.setToGround();
                })
                .splineToConstantHeading(new Vector2d(42,32), Math.toRadians(45))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    //extend outtake
                    outake.extendOutake(1300);
                })
                .waitSeconds(0)
                .forward(11.0)
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
                //relocalize
               /* .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //retract outtake
                    //Pose2d traj2StartPosewOffset = new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY()- frontTagCam.getHorizDisplacement(),drive.getPoseEstimate().getHeading());
                    //drive.setPoseEstimate(traj2StartPosewOffset);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //retract outtake
                })
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(35, 20, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(20,12), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(0)))
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

                })*/

                .lineToLinearHeading(new Pose2d(47,16,Math.toRadians(0)))







                .build();


        TrajectorySequence thirdBlueTraj = drive.trajectorySequenceBuilder(blueStart)
                .lineToLinearHeading(new Pose2d(14, 40, Math.toRadians(270)))
                //.splineToSplineHeading(some position) for spike
                .setReversed(true)
                .back(10) //tune this distance frfr
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //extend outtake
                })
                .splineToConstantHeading(new Vector2d(19, 40), Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(36, 40, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(60, 31), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //open claw
                })
                .waitSeconds(2)
                //relocalize
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
                //relocalize
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
                    //retract outtake
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