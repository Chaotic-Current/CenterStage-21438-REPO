package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Mechanisms.ArmMecNew;
import org.firstinspires.ftc.teamcode.Mechanisms.ClawMech;
import org.firstinspires.ftc.teamcode.Mechanisms.IntakeMech;
import org.firstinspires.ftc.teamcode.Mechanisms.SlideMech;
import org.firstinspires.ftc.teamcode.Pipelines.DetectColor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

@Config
@Autonomous(name = "AA Blue close side fifty pts")
@SuppressWarnings("all")
public class BlueCloseSidefifty extends LinearOpMode {
    private SampleMecanumDrive drive;
    private ElapsedTime timer = new ElapsedTime();
    private OpenCvWebcam frontCam, backCam;
    private IntakeMech intake;
    private ArmMecNew arm;
    private SlideMech slide;
    private ClawMech clawMech;
    private DetectColor detector;
    private Servo wrist;
    private Pose2d[] detectAtThesePoses;

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
    public static double centerLineToLinear1X = 26, centerLineToLinear1Y = 27, centerLineToLinear1Heading = 90, wait1Center = 3;
    public static double centerLineToLinear2Y = 38.5;

    // LEFT
    public static double leftLineToLinear2X = 20, leftLineToLinear2Y = 30, leftLineToLinear2Heading = 90, wait1Left = 3, wait2Left = 1;
    public static double leftLinetoLinear3Y = 38.5;
    public static double leftLinetoLinear1X = 26, leftLinetoLinear1Y = 6, leftLineToLinear1Heading = 30;
    public static double leftBackDist = 10;

    // RIGHT
    public static double rightSpline1deg = -75;
    public static double rightLineToLinear2deg = 90;
    public static double rightSplineTo1X = 26, rightSplineTo1Y = -3.5, splineToLinear1Heading = -80;
    public static double rightLineToLinear2X = 32.75, rightLineToLinear2Y = 30, splineToLinear2Heading = 90, wait1Right = .3, wait2Right = 1;
    public static double rightLineToLinear3Y = 38.5;


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

    public static double yReduction = 29;

    private double thresholdCurrent = 0.5; // threshold for current draw from intake motor

    public static AtomicBoolean t = new AtomicBoolean(false);

    TrajectorySequence autoTrajectory;

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(1430, 1430, 480, 620)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "WebcamFront"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }

    public boolean readyToScan() {
        if (Math.abs(errorY) < 15 && Math.abs(errorY) != 0) {
            // tagUse = 3;
            return true;
        }

        return false;
    }

    private void telemetryAprilTag(int tagUse) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 1) {
                errorX = detection.ftcPose.x;
                errorY = -(detection.ftcPose.y - yReduction);
                errorYaw = detection.ftcPose.yaw;
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, -detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

    public void cameraInit() {
        int width = 160;

        detector = new DetectColor(width, telemetry, new Scalar(140, 255, 255), new Scalar(75, 100, 100), true);
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
        detectAtThesePoses = new Pose2d[4];
        detectAtThesePoses[0] = new Pose2d(20, 30, Math.toRadians(90));
        detectAtThesePoses[1] = new Pose2d(26, 27, Math.toRadians(90));
        detectAtThesePoses[2] = new Pose2d(32.75, 30, Math.toRadians(90));
        drive = new SampleMecanumDrive(hardwareMap);
        arm = new ArmMecNew(hardwareMap);
        slide = new SlideMech(hardwareMap);
        clawMech = new ClawMech(hardwareMap, telemetry);
        intake = new IntakeMech(hardwareMap);
        cameraInit();

        Servo wrist = hardwareMap.get(Servo.class, "WRIST");
        wrist.setPosition(0.5);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        errorX = 0;
        errorY = 0;

        drive.setPoseEstimate(new Pose2d(0, 3, 0));

        DetectColor.ColorLocation e = detector.getLocate();

        while (!isStarted()) {
            e = detector.getLocate();


            telemetry.update();
        }
        frontCam.stopStreaming();
        frontCam.closeCameraDevice();
        initAprilTag();

//|| e == DetectColor.ColorLocation.UNDETECTED
        if (e == DetectColor.ColorLocation.RIGHT || e == DetectColor.ColorLocation.UNDETECTED) {
            // aprilTagPipeline.setTargetTag(3);
            autoTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(frwDistance3)
                    .splineTo(new Vector2d(rightSplineTo1X, rightSplineTo1Y), Math.toRadians(rightSpline1deg))
                    .waitSeconds(1.5)
                    .back(5)

                    .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                        slide.setLowJunction();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(.73, () -> {
                        arm.setExtake();
                    })
                    .waitSeconds(.1)

                    .lineToLinearHeading(new Pose2d(rightLineToLinear2X, rightLineToLinear2Y, Math.toRadians(rightLineToLinear2deg)))

                    .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                        telemetry.addData("test", 1);
                    })

                    .waitSeconds(0.5)

                    .lineToLinearHeading(new Pose2d(rightLineToLinear2X, rightLineToLinear3Y+0.25, Math.toRadians(rightLineToLinear2deg)))

                    .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                        slide.setCustom(930);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                        clawMech.open();
                    })
                    .waitSeconds(1)

                    .back(7)

                    .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                        slide.setLowJunction();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                        arm.setIntake();
                    })

                    .UNSTABLE_addTemporalMarkerOffset(1.7, () -> {
                        clawMech.close();
                    })

                    .UNSTABLE_addTemporalMarkerOffset(3.5, () -> {
                        slide.setIntakeOrGround();
                    })
                    .waitSeconds(1.5)

                    // PARK \\
                    .lineToLinearHeading(new Pose2d(0.5, rightLineToLinear2Y, Math.toRadians(rightLineToLinear2deg)))
                    .waitSeconds(.5)
                    .forward(12)
                    .build();

        } else if (e == DetectColor.ColorLocation.CENTER) {
            tagUse = 2;
            autoTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(centerFrwDistance1)

                    .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                        slide.setLowJunction();
                        // intake.setServosUp();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(.73, () -> {
                        arm.setExtake();
                    })
                    .waitSeconds(.1)

                    .back(centerBackwardsDistance1)

                    .lineToLinearHeading(new Pose2d(centerLineToLinear1X, centerLineToLinear1Y + 2.5, Math.toRadians(centerLineToLinear1Heading)))

                    .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                        // t.set(true);
                    })

                    .waitSeconds(1)

                    .lineToLinearHeading(new Pose2d(centerLineToLinear1X, centerLineToLinear2Y+0.25, Math.toRadians(centerLineToLinear1Heading)))

                    .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                        slide.setCustom(930);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                        clawMech.open();
                    })
                    .waitSeconds(1.5)

                    .back(7)
                    .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                        slide.setLowJunction();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                        arm.setIntake();
                    })

                    .UNSTABLE_addTemporalMarkerOffset(1.7, () -> {
                        clawMech.close();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(2.5, () -> {
                        slide.setIntakeOrGround();

                    })
                    .waitSeconds(2.5)

                    // PARK \\
                    .lineToLinearHeading(new Pose2d(0.5, rightLineToLinear2Y, Math.toRadians(rightLineToLinear2deg)))
                    .waitSeconds(.5)
                    .forward(12) // changed
                    .build();

        } else if (e == DetectColor.ColorLocation.LEFT) {
            tagUse = 1;
            autoTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(leftLinetoLinear1X, leftLinetoLinear1Y, Math.toRadians(leftLineToLinear1Heading)))
                    .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                        slide.setLowJunction();
                    })

                    .UNSTABLE_addTemporalMarkerOffset(.73, () -> {
                        arm.setExtake();
                    })
                    .waitSeconds(.2)
                    .back(leftBackDist)
                    .waitSeconds(.3)

                    .lineToLinearHeading(new Pose2d(leftLineToLinear2X, leftLineToLinear2Y, Math.toRadians(leftLineToLinear2Heading)))
                    .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                        //t.set(true);
                    })
                    .waitSeconds(0.5)
                    .lineToLinearHeading(new Pose2d(leftLineToLinear2X, leftLinetoLinear3Y+0.25, Math.toRadians(leftLineToLinear2Heading)))
                    .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                        slide.setCustom(930);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                        clawMech.open();
                    })
                    .waitSeconds(1.5)

                    .waitSeconds(2)
                    .back(8)

                    .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                        slide.setLowJunction();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                arm.setIntake();
            })

                    .UNSTABLE_addTemporalMarkerOffset(1.7,()->{
                        clawMech.close();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(2.5, () -> {
                        slide.setIntakeOrGround();
                    })
                    .waitSeconds(2)

                    // PARK \\
                    .lineToLinearHeading(new Pose2d(0.5, rightLineToLinear2Y, Math.toRadians(rightLineToLinear2deg)))
                    .waitSeconds(.5)
                    .forward(13) // changed
                    .build();

        } else {
            telemetry.addLine("ruh-roh (´▽`ʃ♡ƪ)");
        }
        waitForStart();

        drive.followTrajectorySequenceAsync(autoTrajectory);

        boolean hasRanOnce = false;
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addLine(errorX + "-x, " + errorY + "-y");
            telemetry.addLine("Encoder Pos: " + drive.getEncoder().getWheelPositions());
            boolean readyToScan = readyToScan();
            telemetry.addLine("Is ready to scan " + readyToScan);

//            if(readyToScan && !hasRanOnce){
//                telemetry.addLine("Offsets added");
//                drive.getEncoder().setErrorX(errorX);
//                drive.getEncoder().setErrorY(errorY);
//                hasRanOnce = true;
//
//            }

            telemetryAprilTag(tagUse);
            telemetry.addLine("" + drive.getPoseEstimate());
            telemetry.addLine(drive.getEncoder().getErrorX() + "-x, " + drive.getEncoder().getErrorY() + "-y");
            drive.update();
            slide.update();
            telemetry.update();

        }
    }
}