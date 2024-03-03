package org.firstinspires.ftc.teamcode.Autos.AutoTests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.Camera;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.MechanismTests.AprilTagCam;
import org.firstinspires.ftc.teamcode.Mechanisms.Outake;
import org.firstinspires.ftc.teamcode.Pipelines.DetectColor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "FSMRelocAuto")
@Config

public class FSMRelocAuto extends LinearOpMode {

    enum States {
        START,

        BACKDROP,

        RETURN,
        IDLE
    }

    States currentState = States.START;
    SampleMecanumDrive drive;

    Camera.PropDetection PropDetection;

    private AprilTagCam frontTagCam;
    private OpenCvWebcam cam;

    DetectColor detector;
    Outake outake;
    Intake intake;
    public static double boardX = 44;
    public static double iterations = 2;

    public static double tagID = 2;
    public static double tagLocation = -35;

    public static double camMultiplier = 0.75;

    public void cameraInit() {
        frontTagCam = new AprilTagCam(hardwareMap,"WebcamFront",tagID);
        frontTagCam.setTelemetry(telemetry);
    }


    public void initialize() {

       // PropDetection = new Camera.PropDetection(hardwareMap,telemetry);
        // PropDetection.runPipeline();
      //  frontTagCam = new AprilTagCam(hardwareMap,"WebcamFront",6);
       // frontTagCam.setTelemetry(telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        cameraInit();
        //outake.closeClawB();
        //outake.closeClawU();
    }


    @Override
    public void runOpMode() {
        int propPos = 2;
        initialize();
        Pose2d blueStart = new Pose2d(14, -60.5, Math.toRadians(90));

        drive.setPoseEstimate(blueStart);

        TrajectorySequence firstBlueTraj = drive.trajectorySequenceBuilder(blueStart)
                .lineToLinearHeading(new Pose2d(14, -35, Math.toRadians(0)))
                .waitSeconds(4)
                .build();

        TrajectorySequence secondBlueTraj = drive.trajectorySequenceBuilder(firstBlueTraj.end())
                .lineToLinearHeading(new Pose2d(45,-35,Math.toRadians(0)))
                .build();

        TrajectorySequence thirdBlueTraj = drive.trajectorySequenceBuilder(secondBlueTraj.end())
                .lineToLinearHeading(new Pose2d(14,-35,Math.toRadians(0)))
                .build();
        //waitForStart();
        //running pipeline after start button is pressed, its a bet, but have to experiement and see if calling runpipline once
        // before the command waitForStart(); causes the camera to be running the entire auto, or stops after init
        //cam.runPipeline();

        //drive.followTrajectorySequenceAsync(camDetection);
        currentState = States.START;

        while (opModeInInit()) {
           // propPos = PropDetection.getPosition();

            //currentDetectionState = cam.getDetectionState();
        }
       // drive.followTrajectorySequenceAsync(firstBlueTraj);
        int currentIteration = 0;
        while (opModeIsActive()) {
            switch (currentState) {
                case START:
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(firstBlueTraj);
                        currentState = States.BACKDROP;
                        currentIteration++;
                    }
                    break;
                case BACKDROP:
                    if (!drive.isBusy()) {
                        //drive.setPoseEstimate();
                        double offset = calculateOffset(tagLocation,drive.getPoseEstimate().getY(),frontTagCam.getHorizDisplacement());
                        Pose2d fixedPose =  new Pose2d(thirdBlueTraj.end().getX(),thirdBlueTraj.end().getY()+offset,thirdBlueTraj.end().getHeading());
                        drive.setPoseEstimate(fixedPose);
                        secondBlueTraj =  drive.trajectorySequenceBuilder(fixedPose)
                                .lineToLinearHeading(new Pose2d(55,-35,Math.toRadians(0)))
                                .build();
                        drive.followTrajectorySequenceAsync(secondBlueTraj);
                        currentIteration++;
                        currentState = States.RETURN;

                    }
                    break;
                case RETURN:
                    if (!drive.isBusy()) {
                        thirdBlueTraj = drive.trajectorySequenceBuilder(secondBlueTraj.end())
                                .lineToLinearHeading(new Pose2d(14,-35,Math.toRadians(0)))
                                .build();
                        drive.followTrajectorySequenceAsync(thirdBlueTraj);
                        if(currentIteration>iterations)
                            currentState = States.IDLE;
                        else
                            currentState = States.BACKDROP;
                    }
                    break;
                case IDLE:

                    break;
                    
            }
            drive.update();
            frontTagCam.aprilTagUpdate();

        }
    }
    public static double calculateOffset(double tagPosition, double currentPosition, double camData){
        return camMultiplier*(camData-(currentPosition-tagPosition));
    }
}