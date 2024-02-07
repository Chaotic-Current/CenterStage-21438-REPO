package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Mechanisms.ArmMecNew;
import org.firstinspires.ftc.teamcode.Mechanisms.ClawMech;
import org.firstinspires.ftc.teamcode.Mechanisms.IntakeMech;
import org.firstinspires.ftc.teamcode.Mechanisms.SlideMech;
import org.firstinspires.ftc.teamcode.Pipelines.DetectColor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous (name = "AA Red Close Side")
public class RedClose_50 extends LinearOpMode {

    public static int location = 1;
    private SampleMecanumDrive drive;
    private OpenCvWebcam frontCam, backCam;
    private ArmMecNew arm;
    private SlideMech slide;
    private ClawMech clawMech;
    private Servo wrist;
    private IntakeMech intakeMech;
    private DetectColor detector; //This will be out of the frame for know, along with the april tag pipeline
    public static double frwDistance1 = 30;
    public static double backwardsDistance1 = 12;
    public static double frwDistance2 = 5;
    public static double frwDistance3 = 12;
    public static double wait01 = 1;
    public static double wait02 = 1;
    public static double spline1deg = 75;
    public static double backdist2 = 8;
    public static double spline2deg = 90;
    public static double frw = 10.25;

    public static double linetoLinear1X = 26, linetoLinear1Y = -6, lineToLinear1Heading = -30;
    public static double splineToLinear1X = 26, splineToLinear1Y = 5.25, splineToLinear1Heading = 80;
    public static double splineToLinear2X = 29, splineToLinear2Y = -39.5, splineToLinear2Heading = -90, wait1 = 3;
    public static double splineToLinear3X = 26, splineToLinear3Y = -39.5, splineToLinear3Heading = -90, wait2 = 3;
    public static double splineToLinear4X = 20, splineToLinear4Y = -39.5, splineToLinear4Heading = -90, wait3 = 3;
    public static double frparkX = 4, frparkY=-38, frparkHeading = -90;

    public static double parkX = 4, parkY = 37, parkHeading = 90;
    public static double degree = 90;

    TrajectorySequence firstMove;
    TrajectorySequence park;

    public void cameraInit(){
        int width = 160;


        detector = new DetectColor(width, telemetry, new Scalar(10,255,255),new Scalar(2,100,100));

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

    public void initialize(){
        drive = new SampleMecanumDrive(hardwareMap);
        arm = new ArmMecNew(hardwareMap);
        slide = new SlideMech(hardwareMap);
        clawMech = new ClawMech(hardwareMap,telemetry);
        //intakeMech = new IntakeMech(hardwareMap);
        wrist = hardwareMap.get(Servo.class,"WRIST");
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
            if (e != null) {
                //telemetry.addLine("in loop " + e.name());
            }
            telemetry.update();
        }
        frontCam.stopStreaming();
        frontCam.closeCameraDevice();
        telemetry.addLine(e.name());
        telemetry.update();



        if (e == DetectColor.ColorLocation.LEFT || e == DetectColor.ColorLocation.UNDETECTED) {

            firstMove = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(frwDistance3)
                    .splineTo(new Vector2d(splineToLinear1X, splineToLinear1Y+0.2), Math.toRadians(spline1deg))
                    .waitSeconds(0.75)
                    .back(5)
                    .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                        slide.setLowJunction();
                    })

                    .UNSTABLE_addTemporalMarkerOffset(.73, () -> {
                        arm.setExtake();
                    })
                    .waitSeconds(.1)
                    .lineToLinearHeading(new Pose2d(splineToLinear2X+3, splineToLinear2Y, Math.toRadians(-spline2deg)))
                    .waitSeconds(0.25)
                    //.forward(8)


                    .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                        slide.setCustom(980);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                        clawMech.open();
                    })

                    .waitSeconds(0.75)
                    .back(7)

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
                    .waitSeconds(3.25)
                    .strafeRight(28)
                    .waitSeconds(.5)
                    .forward(11.5)
                    .build();


        } else if (e == DetectColor.ColorLocation.CENTER) {
            firstMove = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(frwDistance1)
                    .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                        slide.setLowJunction();
                    })

                    .UNSTABLE_addTemporalMarkerOffset(.73, () -> {
                        arm.setExtake();
                    })
                    .waitSeconds(.1)

                    .back(backwardsDistance1)

                    .lineToLinearHeading(new Pose2d(splineToLinear3X, splineToLinear3Y+.2, Math.toRadians(splineToLinear3Heading)))

                    //.forward(4.5)

                    .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                        slide.setCustom(980);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                        clawMech.open();
                    })
                    .waitSeconds(1)

                    .waitSeconds(2)
                    .back(7)

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
                    .strafeRight(24)
                    .waitSeconds(.5)
                    .forward(11)
                    .build();


        } else if(e == DetectColor.ColorLocation.RIGHT) {
            firstMove = drive.trajectorySequenceBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(linetoLinear1X, -5.4, Math.toRadians(lineToLinear1Heading)))
                    .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                        slide.setLowJunction();
                    })

                    .UNSTABLE_addTemporalMarkerOffset(.73, () -> {
                        arm.setExtake();
                    })
                    .waitSeconds(1)
                    .back(backdist2)
                    .waitSeconds(.3)
                    .lineToLinearHeading(new Pose2d(20, splineToLinear4Y+.5, Math.toRadians(splineToLinear4Heading)))
                    //.forward(frw)

                    .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                        slide.setCustom(980);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                        clawMech.open();
                    })
                    .waitSeconds(1)

                    .waitSeconds(2)
                    .back(7)

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
                    .strafeRight(18)
                    .waitSeconds(.5)
                    .forward(11)
                    .build();

        }else {
            telemetry.addLine("ruh-roh (´▽`ʃ♡ƪ)");
        }
        waitForStart();


        drive.followTrajectorySequenceAsync(firstMove);

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            slide.update(telemetry);
            telemetry.update();
        }
    }
}