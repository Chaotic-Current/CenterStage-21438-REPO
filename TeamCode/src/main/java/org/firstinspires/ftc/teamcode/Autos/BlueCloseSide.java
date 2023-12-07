package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MechanismTemplates.ArmPID;
import org.firstinspires.ftc.teamcode.MechanismTemplates.ClawMech;
import org.firstinspires.ftc.teamcode.MechanismTemplates.SlideMech;
import org.firstinspires.ftc.teamcode.Pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Pipelines.DetectColor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous
public class BlueCloseSide extends LinearOpMode {

    private SampleMecanumDrive drive;
    private OpenCvWebcam frontCam, backCam;
    private ArmPID arm;
    private SlideMech slide;
    private ClawMech clawMech;
    private DetectColor detector;
    private AprilTagDetectionPipeline aprilTagPipeline;
    public static double centerFrwDistance1 = 30;
    public static double centerBackwardsDistance1 = 12;
    public static double frwDistance2 = 5;
    public static double frwDistance3 = 12;
    public static double wait01 = 1;
    public static double wait02 = 1;
    public static double rightSpline1deg = -75;
    public static double leftBackDist = 6;
    public static double rightLineToLinear2deg = -90;

    public static double leftLinetoLinear1X = 26, leftLinetoLinear1Y = 6, leftLineToLinear1Heading = 30;
    public static double rightSplineTo1X = 26, rightSplineTo1Y = -2.5, splineToLinear1Heading = -80;
    public static double rightLineToLinear2X = 32, rightLineToLinear2Y = 37.5, splineToLinear2Heading = 90, wait1Right = .3, wait2Right = 1;
    public static double rightLineToLinear3Y = 41;
    public static double centerLineToLinear1X = 26, centerLineToLinear1Y = 30, centerLineToLinear1Heading = 90, wait1Center = 3;
    public static double centerLineToLinear2Y = 39.5;
    public static double leftLineToLinear2X = 20, leftLineToLinear2Y = 33.5, leftLineToLinear2Heading = 90, wait1Left = 3, wait2Left = 1;
    public static double leftLinetoLinear3Y = 40.5;
    public static double parkX = 4, parkY = 37, parkHeading = 90;
    public static double degree = 90;

    TrajectorySequence autoTrajectory;

    public void cameraInit() {
        int width = 160;

        detector = new DetectColor(width, telemetry, new Scalar(140, 255, 255), new Scalar(75, 100, 100));
        aprilTagPipeline = new AprilTagDetectionPipeline();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // backCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "WebcamBack"), cameraMonitorViewId);
        frontCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "WebcamFront"), cameraMonitorViewId);
        // backCam.setPipeline(detector);
        frontCam.setPipeline(detector);

        // backCam.setMillisecondsPermissionTimeout(2500);
        frontCam.setMillisecondsPermissionTimeout(2500);
        /*backCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                telemetry.addLine("started");
                backCam.startStreaming(160, 120, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("not open");
            }
        });*/
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
        drive = new SampleMecanumDrive(hardwareMap);
        arm = new ArmPID(hardwareMap);
        slide = new SlideMech(hardwareMap);
        clawMech = new ClawMech(hardwareMap, telemetry);
        cameraInit();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        drive.setPoseEstimate(new Pose2d());

        DetectColor.ColorLocation e = detector.getLocate();
        ElapsedTime time = new ElapsedTime();
        while (e == null || time.milliseconds() <= 1000) {
            e = detector.getLocate();
            if (e != null) {
                telemetry.addLine("in loop " + e.name());
                telemetry.update();
            }
            if (e == null)
                time.reset();
        }

        frontCam.stopStreaming();
        telemetry.addLine(e.name());
        telemetry.update();

        frontCam.setPipeline(aprilTagPipeline);
        frontCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                telemetry.addLine("started apriltag");
                frontCam.startStreaming(160, 120, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addLine("not open");
            }
        });



        if (e == DetectColor.ColorLocation.RIGHT || e == DetectColor.ColorLocation.UNDETECTED) {
            //aprilTagPipeline.setTargetTag(3);

            autoTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(frwDistance3)
                    .splineTo(new Vector2d(rightSplineTo1X, rightSplineTo1Y), Math.toRadians(rightSpline1deg))
                    .waitSeconds(3)
                    .back(5)

                    .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                        slide.setLowJunction();
                    })

                    .UNSTABLE_addTemporalMarkerOffset(.73, () -> {
                        arm.setExtake(0.0);
                    })
                    .waitSeconds(.1)
                    .lineToLinearHeading(new Pose2d(rightLineToLinear2X, rightLineToLinear2Y, Math.toRadians(rightLineToLinear2deg)))
                    .UNSTABLE_addTemporalMarkerOffset(0.15,() ->{
                        double newX = drive.getPoseEstimate().getX() + aprilTagPipeline.getErrorX();
                       // double newY = drive.getPoseEstimate().getY() - (aprilTagPipeline.getErrorY() - 4);
                        double newAngle = drive.getPoseEstimate().getHeading() - Math.toRadians(aprilTagPipeline.getErrorYaw());
                        drive.setPoseEstimate(new Pose2d(newX, drive.getPoseEstimate().getY(),newAngle));
                   })
                    .waitSeconds(0.5)
                    .lineToLinearHeading(new Pose2d(rightLineToLinear2X,rightLineToLinear3Y,Math.toRadians(rightLineToLinear2deg)))
                    .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                        slide.setCustom(1000);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                        clawMech.open();
                    })
                    .waitSeconds(2)
                    .back(5)

                    .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                        slide.setLowJunction();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                        arm.setIntake();
                        clawMech.close();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(2.5, () -> {
                        slide.setIntakeOrGround();
                    })
                    .waitSeconds(5)
                    .strafeLeft(28)
                    .waitSeconds(.5)
                    .forward(11)
                    .build();


        } else if (e == DetectColor.ColorLocation.CENTER) {
            //aprilTagPipeline.setTargetTag(2);
            autoTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(centerFrwDistance1)
                    .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                        slide.setLowJunction();
                        telemetry.addData("slides?", 0);
                    })

                    .UNSTABLE_addTemporalMarkerOffset(.73, () -> {
                        arm.setExtake(0.0);
                        telemetry.addData("arm?", 0);
                    })
                    .waitSeconds(.1)

                    .back(centerBackwardsDistance1)

                    .lineToLinearHeading(new Pose2d(centerLineToLinear1X, centerLineToLinear1Y, Math.toRadians(centerLineToLinear1Heading)))
                    .UNSTABLE_addTemporalMarkerOffset(0.15,() ->{
                        double newX = drive.getPoseEstimate().getX() + aprilTagPipeline.getErrorX();
                        // double newY = drive.getPoseEstimate().getY() - (aprilTagPipeline.getErrorY() - 4);
                        double newAngle = drive.getPoseEstimate().getHeading() - Math.toRadians(aprilTagPipeline.getErrorYaw());
                        drive.setPoseEstimate(new Pose2d(newX, drive.getPoseEstimate().getY(),newAngle));
                    })
                    .waitSeconds(0.5)
                    .lineToLinearHeading(new Pose2d(centerLineToLinear1X,centerLineToLinear2Y,Math.toRadians(centerLineToLinear1Heading)))
                    .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                        slide.setCustom(1000);
                        telemetry.addData("slides again?", 0);

                    })
                    .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                        clawMech.open();
                    })
                    .waitSeconds(0.1)

                    .waitSeconds(2)
                    .back(5)

                    .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                        slide.setLowJunction();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                        arm.setIntake();
                        clawMech.close();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(2.5, () -> {
                        slide.setIntakeOrGround();
                    })
                    .waitSeconds(5)
                    .strafeLeft(24)
                    .waitSeconds(.5)
                    .forward(11)
                    .build();


        } else if (e == DetectColor.ColorLocation.LEFT) {
            //aprilTagPipeline.setTargetTag(1);
            autoTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(leftLinetoLinear1X, leftLinetoLinear1X, Math.toRadians(leftLineToLinear1Heading)))
                    .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                        slide.setLowJunction();
                    })

                    .UNSTABLE_addTemporalMarkerOffset(.73, () -> {
                        arm.setExtake(0.0);
                    })
                    .waitSeconds(.2)
                    .back(leftBackDist)
                    .waitSeconds(.3)

                    .lineToLinearHeading(new Pose2d(leftLineToLinear2X, leftLineToLinear2Y, Math.toRadians(leftLineToLinear2Heading)))
                    .UNSTABLE_addTemporalMarkerOffset(0.15,() ->{
                        double newX = drive.getPoseEstimate().getX() + aprilTagPipeline.getErrorX();
                        // double newY = drive.getPoseEstimate().getY() - (aprilTagPipeline.getErrorY() - 4);
                        double newAngle = drive.getPoseEstimate().getHeading() - Math.toRadians(aprilTagPipeline.getErrorYaw());
                        drive.setPoseEstimate(new Pose2d(newX, drive.getPoseEstimate().getY(),newAngle));
                    })
                    .waitSeconds(0.5)
                    .lineToLinearHeading(new Pose2d(leftLineToLinear2X,leftLinetoLinear3Y,Math.toRadians(leftLineToLinear2Heading)))
                    .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                        slide.setCustom(1000);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                        clawMech.open();
                    })
                    .waitSeconds(0.1)

                    .waitSeconds(2)
                    .back(5)

                    .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                        slide.setLowJunction();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                        arm.setIntake();
                        clawMech.close();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(2.5, () -> {
                        slide.setIntakeOrGround();
                    })
                    .waitSeconds(5)
                    .strafeLeft(20)
                    .waitSeconds(.5)
                    .forward(11)
                    .build();

        } else {
            telemetry.addLine("ruh roh");
        }

        waitForStart();
        telemetry.update();
        telemetry.addLine(e.name());
        telemetry.update();


        drive.followTrajectorySequenceAsync(autoTrajectory);

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            slide.update(telemetry);
            arm.update(telemetry, new ElapsedTime());

        }
    }
}