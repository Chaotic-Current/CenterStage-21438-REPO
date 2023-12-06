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
public class BlueCloseSide extends LinearOpMode{
    
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
    public static double rightLineToLinear2X = 29, rightLineToLinear2Y = 30, splineToLinear2Heading = 90, wait1Right = 3, wait2Right = 1;
    public static double rightLineToLinear3Y = 34;
    public static double centerLineToLinear1X = 26, centerLineToLinear1Y = 30, centerLineToLinear1Heading = 90, wait1Center = 3;
    public static double centerLineToLinear2Y = 38;
    public static double leftLineToLinear2X = 20, leftLineToLinear2Y = 30.5, leftLineToLinear2Heading = 90, wait1Left = 3, wait2Left = 1;
    public static double leftLinetoLinear3Y = 33.5;
    public static double parkX = 4, parkY = 37, parkHeading = 90;
    public static double degree = 90;

    TrajectorySequence autoTrajectory;

    public void cameraInit() {
        int width = 160;

        detector = new DetectColor(width, telemetry, new Scalar(140, 255, 255), new Scalar(75, 100, 100));

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

//        frontCam.setPipeline(aprilTagPipeline);
//        frontCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                telemetry.addLine("started apriltag");
//                frontCam.startStreaming(160, 120, OpenCvCameraRotation.UPRIGHT);
//            }
//            @Override
//            public void onError(int errorCode) {
//                telemetry.addLine("not open");
//            }
//        });

//       I added what I think would be proper points for mechanisms to do what they need to do, but I commented them out just for now

        if (e == DetectColor.ColorLocation.RIGHT || e == DetectColor.ColorLocation.UNDETECTED){

            autoTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(frwDistance3)
                    .splineTo(new Vector2d(rightSplineTo1X, rightSplineTo1Y), Math.toRadians(rightSpline1deg))
                    .waitSeconds(0.5)
                    .back(5)
                    .UNSTABLE_addTemporalMarkerOffset(wait1Right, () -> {
                        slide.setLowJunction();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(wait1Right + 1, () -> {
                        arm.setExtakeOrIntake();
                    })
                    .lineToLinearHeading(new Pose2d(rightLineToLinear2X, rightLineToLinear2Y, Math.toRadians(rightLineToLinear2deg)))

//  don't do anything with this
//                    .UNSTABLE_addTemporalMarkerOffset(0.5,() ->{
//                        double newX = drive.getPoseEstimate().getX() + aprilTagPipeline.getErrorX();
//                        double newY = drive.getPoseEstimate().getY() - (aprilTagPipeline.getErrorY() - 4);
//                        double newAngle = drive.getPoseEstimate().getHeading() + Math.toRadians(aprilTagPipeline.getErrorYaw());
//                        drive.setPoseEstimate(new Pose2d(newX, newY,newAngle));
//                    })
//                    .waitSeconds(0.7)

                    .lineToLinearHeading(new Pose2d(rightLineToLinear2X, rightLineToLinear3Y, Math.toRadians(rightLineToLinear2deg)))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        clawMech.open();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(wait2Right, () -> {
                        arm.setIntake();
                        clawMech.close();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(wait2Right + 1.5, () -> {
                        slide.setIntakeOrGround();
                    })
                    .waitSeconds(15)
                    .build();


        }
        else if (e == DetectColor.ColorLocation.CENTER) {
            autoTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(centerFrwDistance1)
                    .back(centerBackwardsDistance1)
                    .UNSTABLE_addTemporalMarkerOffset(wait1Center, () -> {
                        slide.setLowJunction();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(wait1Center + 1, () -> {
                        arm.setExtakeOrIntake();
                    })
                    .lineToLinearHeading(new Pose2d(centerLineToLinear1X, centerLineToLinear1Y, Math.toRadians(centerLineToLinear1Heading)))
                    //don't do anything with this
//                    .UNSTABLE_addTemporalMarkerOffset(0.5,() ->{
//                        double newX = drive.getPoseEstimate().getX() + aprilTagPipeline.getErrorX();
//                        double newY = drive.getPoseEstimate().getY() - (aprilTagPipeline.getErrorY() - 4);
//                        double newAngle = drive.getPoseEstimate().getHeading() + Math.toRadians(aprilTagPipeline.getErrorYaw());
//                        drive.setPoseEstimate(new Pose2d(newX, newY,newAngle));
//                    })
//                  .waitSeconds(0.7)
                    .lineToLinearHeading(new Pose2d(centerLineToLinear1X, centerLineToLinear2Y, Math.toRadians(centerLineToLinear1Heading)))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        clawMech.open();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(wait1Center, () -> {
                        arm.setIntake();
                        clawMech.close();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(wait1Center + 1.5, () -> {
                        slide.setIntakeOrGround();
                    })
                    .waitSeconds(15)
                    .build();

        }
        else if (e == DetectColor.ColorLocation.LEFT) {
            autoTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(leftLinetoLinear1X, leftLinetoLinear1Y, Math.toRadians(leftLineToLinear1Heading)))
                    .back(leftBackDist)
                    .UNSTABLE_addTemporalMarkerOffset(wait1Left, () -> {
                        slide.setLowJunction();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(wait1Left + 1, () -> {
                        arm.setExtakeOrIntake();
                    })
                    .lineToLinearHeading(new Pose2d(leftLineToLinear2X, leftLineToLinear2Y, Math.toRadians(leftLineToLinear2Heading)))
                    //don't do anything with this
//                    .UNSTABLE_addTemporalMarkerOffset(0.5,() ->{
//                        double newX = drive.getPoseEstimate().getX() + aprilTagPipeline.getErrorX();
//                        double newY = drive.getPoseEstimate().getY() - (aprilTagPipeline.getErrorY() - 4);
//                        double newAngle = drive.getPoseEstimate().getHeading() + Math.toRadians(aprilTagPipeline.getErrorYaw());
//                        drive.setPoseEstimate(new Pose2d(newX, newY,newAngle));
//                    })
//                    .waitSeconds(0.7)
                    .lineToLinearHeading(new Pose2d(leftLineToLinear2X,leftLinetoLinear3Y,Math.toRadians(leftLineToLinear2Heading)))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        clawMech.open();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(wait1Left, () -> {
                        arm.setIntake();
                        clawMech.close();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(wait1Left + 1.5, () -> {
                        slide.setIntakeOrGround();
                    })
                    .waitSeconds(15)
                    .build();

        }
        else {
            telemetry.addLine("ruh roh");
        }
            waitForStart();
            telemetry.update();
            telemetry.addLine(e.name());
            telemetry.update();

            frontCam.stopStreaming();

            drive.followTrajectorySequence(autoTrajectory);

            while (opModeIsActive() && !isStopRequested()) {
                drive.update();
                slide.update(telemetry);
                arm.update(telemetry, new ElapsedTime());

            }
        }
    }
