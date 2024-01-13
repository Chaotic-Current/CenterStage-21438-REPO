package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MechanismTemplates.ArmMecNew;
import org.firstinspires.ftc.teamcode.MechanismTemplates.ClawMech;
import org.firstinspires.ftc.teamcode.MechanismTemplates.IntakeMech;
import org.firstinspires.ftc.teamcode.MechanismTemplates.SlideMech;
//import org.firstinspires.ftc.teamcode.Pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Pipelines.DetectColor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous
public class BlueFarSide extends LinearOpMode {
    private SampleMecanumDrive drive;
    private ElapsedTime timer = new ElapsedTime();
    private OpenCvWebcam frontCam, backCam;
    private IntakeMech intake;
    private ArmMecNew arm;
    private SlideMech slide;
    private ClawMech clawMech;
    private DetectColor detector;
    private Servo wrist;

    // private AprilTagDetectionPipeline aprilTagPipeline;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private int tagUse;
    public static double errorX;
    public static double errorY;
    public static double errorYaw;

    // CENTER
    public static double centerFrwDistance1 = 30;
    public static double centerBackwardsDistance1 = 12;
    public static double frwDistance2 = 5;
    public static double frwDistance3 = 12;
    public static double wait01 = 1;
    public static double wait02 = 1;
    public static double centerLineToLinear1X = 26, centerLineToLinear1Y = 30, centerLineToLinear1Heading = 90, wait1Center = 3;
    public static double centerLineToLinear2Y = 41;

    // LEFT
    public static double leftLineToLinear2X = 20, leftLineToLinear2Y = 33.5, leftLineToLinear2Heading = 90, wait1Left = 3, wait2Left = 1;
    public static double leftLinetoLinear3Y = 40.5;
    public static double leftLinetoLinear1X = 26, leftLinetoLinear1Y = 6, leftLineToLinear1Heading = 30;
    public static double leftBackDist = 10;

    // RIGHT
    public static double rightSpline1deg = -75;
    public static double rightLineToLinear2deg = 90;
    public static double rightSplineTo1X = 26, rightSplineTo1Y = -3, splineToLinear1Heading = -80;
    public static double rightLineToLinear2X = 32.75, rightLineToLinear2Y = 37.5, splineToLinear2Heading = 90, wait1Right = .3, wait2Right = 1;
    public static double rightLineToLinear3Y = 40.25;


    // OTHER
    public static double parkX = 4, parkY = 37, parkHeading = 90;
    public static double degree = 90;
    double errorx, errory, errorheading;
    public static double toStackLinetoLinear1X, toStackLinetoLinear1Y = -69.75, toStackLineToLinear1Heading;
    public static double toStackLinetoLinear2X, toStackLinetoLinear2Y, toStackLineToLinear2Heading;
    public static double toBackBoardLinetoLinear1X, toBackBoardLinetoLinear1Y, toBackBoardLinetoLinear1Heading;
    public static double toBackBoardLinetoLinear2X, toBackBoardLinetoLinear2Y, toBackBoardLinetoLinear2Heading;
    String x;
    private int numOfPixels;

    private double thresholdCurrent = 0.5; // threshold for current draw from intake motor

    public static AtomicBoolean t = new AtomicBoolean(false);

    TrajectorySequence autoTrajectory;

    public void cameraInit() {
        int width = 160;

        detector = new DetectColor(width, telemetry, new Scalar(140, 255, 255), new Scalar(75, 100, 100),false);
        //  aprilTagPipeline = new AprilTagDetectionPipeline();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // backCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "WebcamBack"), cameraMonitorViewId);
        frontCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "WebcamFront"), cameraMonitorViewId);
        // backCam.setPipeline(detector);
        frontCam.setPipeline(detector);

        // backCam.setMillisecondsPermissionTimeout(2500);
        frontCam.setMillisecondsPermissionTimeout(2500);
        frontCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                telemetry.addLine("started");
                frontCam.startStreaming(160, 120, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("not open");
            }
        });
    }

    public void initialize() {
        //intake = new IntakeMech(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        arm = new ArmMecNew(hardwareMap);
        slide = new SlideMech(hardwareMap);
        clawMech = new ClawMech(hardwareMap, telemetry);
        intake = new IntakeMech(hardwareMap);
        wrist = hardwareMap.get(Servo.class, "WRIST");
        wrist.setPosition(0.5);
        cameraInit();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        drive.setPoseEstimate(new Pose2d(0,-3,0));

        DetectColor.ColorLocation e = detector.getLocate();

        while (!isStarted()) {
            e = detector.getLocate();


            telemetry.update();
        }
        frontCam.stopStreaming();
        frontCam.closeCameraDevice();

//|| e == DetectColor.ColorLocation.UNDETECTED
        if (e == DetectColor.ColorLocation.RIGHT ) {
            // aprilTagPipeline.setTargetTag(3);
            autoTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(frwDistance3)
                    .splineTo(new Vector2d(rightSplineTo1X, rightSplineTo1Y), Math.toRadians(rightSpline1deg))
                    .back(7)
                    .waitSeconds(1.5)
                    .build();

        } else if (e == DetectColor.ColorLocation.CENTER) {
            tagUse = 2;
            autoTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(centerFrwDistance1)
                    .back(7)
                    .build();

        } else if (e == DetectColor.ColorLocation.LEFT || e == DetectColor.ColorLocation.UNDETECTED) {
            autoTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(leftLinetoLinear1X, leftLinetoLinear1Y, Math.toRadians(leftLineToLinear1Heading)))
                    .back(7)
                    .build();

        } else {
            telemetry.addLine("ruh-roh (´▽`ʃ♡ƪ)");
        }
        waitForStart();

        drive.followTrajectorySequenceAsync(autoTrajectory);

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            telemetry.update();
        }
    }
}
