package org.firstinspires.ftc.teamcode.Deprecated;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Mechanisms.ArmMecNew;
import org.firstinspires.ftc.teamcode.Mechanisms.ClawMech;
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
@Autonomous(name = "OneTrajectoryBlueCloseSide")
@Disabled
public class OneTrajectoryBlueCloseSide extends LinearOpMode {

    public static int location = 1;
    private SampleMecanumDrive drive;
    private OpenCvWebcam frontCam, backCam;
    private ArmMecNew arm;
    private SlideMech slide;
    private ClawMech clawMech;
    private DetectColor detector; //This will be out of the frame for know, along with the april tag pipeline
    public static double frwDistance1 = 30;
    public static double backwardsDistance1 = 12;
    public static double frwDistance2 = 7;
    public static double wait01 = 1;
    public static double wait02 = 1;
    public static double linetoLinear1X = 24, linetoLinear1Y = 24, lineToLinear1Heading = 30;
    public static double splineToLinear1X = 24, splineToLinear1Y = -24, splineToLinear1Heading = -80;
    public static double splineToLinear2X = 17.5, splineToLinear2Y = 33, splineToLinear2Heading = 90, wait1 = 3;
    public static double splineToLinear3X = 26, splineToLinear3Y = 38, splineToLinear3Heading = 90, wait2 = 3;
    public static double splineToLinear4X = 8, splineToLinear4Y = 32, splineToLinear4Heading = 90, wait3 = 3;

    public static double parkX = 4, parkY = 37, parkHeading = 90;
    public static double degree = 90;

    TrajectorySequence firstMove;
    TrajectorySequence moveToBackboard;
    TrajectorySequence park;

    public void cameraInit(){
        int width = 160;


        detector = new DetectColor(width, telemetry, new Scalar(140,255,255),new Scalar(75,100,100));

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
        cameraInit();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        drive.setPoseEstimate(new Pose2d());

        DetectColor.ColorLocation e = detector.getLocate();
        while (e == null || e == DetectColor.ColorLocation.UNDETECTED){
            e = detector.getLocate();
            if(e != null) {
                telemetry.addLine("in loop " + e.name());
                telemetry.update();
            }
        }

        telemetry.addLine(e.name());
        telemetry.update();

        //I added what I think would be proper points for mechanisms to do what they need to do, but I commented them out just for now
        if (e == DetectColor.ColorLocation.RIGHT) {

            firstMove = drive.trajectorySequenceBuilder(new Pose2d())
                    .splineToLinearHeading(new Pose2d(splineToLinear1X, splineToLinear1Y, Math.toRadians(splineToLinear1Heading)), Math.toRadians(0))
                    .build();

            moveToBackboard = drive.trajectorySequenceBuilder(firstMove.end())
                    .splineToLinearHeading(new Pose2d(splineToLinear2X, splineToLinear2Y, Math.toRadians(splineToLinear2Heading)), Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(wait1, () -> {
                        //slide.setLowJunction();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(wait1 + 1, () -> {
                        //arm.setExtakeOrIntake();
                    })
                    .waitSeconds(4)
                    .UNSTABLE_addTemporalMarkerOffset(wait1 + 2, () -> {
                        //clawMech.setBothOpen(false);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(wait1 + 2.5, () -> {
                        //arm.setExtakeOrIntake();
                        //clawMech.getClaw().setPosition(clawMech.close);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(wait1 + 3.5, () -> {
                        //slide.setIntakeOrGround();
                    })
                    .build();


        } else if (e == DetectColor.ColorLocation.CENTER) {
            firstMove = drive.trajectorySequenceBuilder(new Pose2d())
                    // first move
                    .forward(frwDistance1)
                    .back(backwardsDistance1)

                    // move to backdrop
                    .lineToLinearHeading(new Pose2d(splineToLinear3X, splineToLinear3Y, Math.toRadians(splineToLinear3Heading)))
                    .UNSTABLE_addTemporalMarkerOffset(wait2, () -> {
                        slide.setLowJunction();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(wait2 + 1, () -> {
                        arm.setExtake();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(wait2 + 3, () -> {
                        slide.setCustom(1290);
                    })
                    .waitSeconds(5)
                    .UNSTABLE_addTemporalMarkerOffset(wait2 + 2, () -> {
                        clawMech.open();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(wait2 + 3, () -> {
                        arm.setIntake();
                        clawMech.close();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(wait2 + 6, () -> {
                        slide.setIntakeOrGround();
                    })
                    .waitSeconds(15)

                  // park
                  .lineToLinearHeading(new Pose2d(parkX,parkY,parkHeading))
                        .forward(frwDistance2)
                        .build();


        } else if(e == DetectColor.ColorLocation.LEFT) {
            firstMove = drive.trajectorySequenceBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(linetoLinear1X, linetoLinear1Y, Math.toRadians(lineToLinear1Heading)))
                    .build();

            moveToBackboard = drive.trajectorySequenceBuilder(firstMove.end())
                    .splineToLinearHeading(new Pose2d(splineToLinear4X, splineToLinear4Y, Math.toRadians(splineToLinear4Heading)), Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(wait3, () -> {
                        //slide.setLowJunction();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(wait3 + 1, () -> {
                        //arm.setExtakeOrIntake();
                    })
                    .waitSeconds(4)
                    .UNSTABLE_addTemporalMarkerOffset(wait3 + 2, () -> {
                        //clawMech.setBothOpen(false);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(wait3 + 2.5, () -> {
                        //arm.setExtakeOrIntake();
                        //clawMech.getClaw().setPosition(clawMech.close);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(wait3 + 3.5, () -> {
                        //slide.setIntakeOrGround();
                    })
                    .build();

        }else {
            telemetry.addLine("its curtins");
            firstMove = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(frwDistance1)
                    .waitSeconds(wait01)
                    .back(backwardsDistance1)
                    .waitSeconds(wait02)
                    .turn(Math.toRadians(degree))
                    .forward(frwDistance2)
                    .build();


                /*
                moveBack = drive.trajectorySequenceBuilder(new Pose2d())
                        .back(backwardsDistance1)
                        .build();
                        */
            moveToBackboard = drive.trajectorySequenceBuilder(firstMove.end())
                    .splineToLinearHeading(new Pose2d(splineToLinear3X,splineToLinear3Y,Math.toRadians(splineToLinear3Heading)), Math.toRadians(0))
                    .build();
        }

        waitForStart();
        telemetry.update();
        telemetry.addLine(e.name());
        telemetry.update();

        frontCam.stopStreaming();

        drive.followTrajectorySequenceAsync(firstMove);

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            slide.update();


        }
    }
}
