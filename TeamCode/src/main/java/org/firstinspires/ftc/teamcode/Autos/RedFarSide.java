package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MechanismTemplates.ArmMecNew;
import org.firstinspires.ftc.teamcode.MechanismTemplates.ClawMech;
import org.firstinspires.ftc.teamcode.MechanismTemplates.SlideMech;
import org.firstinspires.ftc.teamcode.Pipelines.DetectColor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class RedFarSide extends LinearOpMode {

    private SampleMecanumDrive drive;
    private OpenCvWebcam frontCam, backCam;
    private ArmMecNew arm;
    private SlideMech slide;
    private ClawMech clawMech;
    private DetectColor detector; //This will be out of the frame for know, along with the april tag pipeline
    //TODO all these values are just place holders so the code doesn't cause an error, delete this comment after you see it
    public static double frwDistance1 = 30;
    public static double backwardsDistance1 = 12;
    public static double frwDistance2 = 5;
    public static double frwDistance3 = 12;
    public static double wait01 = 1;
    public static double wait02 = 1;
    public static double spline1deg = 75;
    public static double backdist2 = 6;
    public static double spline2deg = 90;
    public static double frw = 10.25;

    public static double linetoLinear1X = 26, linetoLinear1Y = -6, lineToLinear1Heading = -30;
    public static double splineToLinear1X = 26, splineToLinear1Y = 1.75, splineToLinear1Heading = 80;
    public static double splineToLinear2X = 29, splineToLinear2Y = -34, splineToLinear2Heading = -90, wait1 = 3;
    public static double splineToLinear3X = 26, splineToLinear3Y = -38, splineToLinear3Heading = -90, wait2 = 3;
    public static double splineToLinear4X = 20, splineToLinear4Y = -33.5, splineToLinear4Heading = -90, wait3 = 3;
    public static double frparkX = 4, frparkY=-38, frparkHeading = -90;

    public static double parkX = 4, parkY = 37, parkHeading = 90;
    public static double degree = 90;

    TrajectorySequence firstMove;
    TrajectorySequence moveToBackboard;
    TrajectorySequence park;

    //Fuck this shit
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

        cameraInit();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        drive.setPoseEstimate(new Pose2d());


        DetectColor.ColorLocation e = detector.getLocate();
        ElapsedTime time = new ElapsedTime();
        while (e == null && time.milliseconds() <= 5000) {
            e = detector.getLocate();
            if (e != null) {
                telemetry.addLine("in loop " + e.name());
                telemetry.update();
            }

            if (e == null && time.milliseconds() >= 3500)
                e = DetectColor.ColorLocation.UNDETECTED;

        }
        frontCam.stopStreaming();
        telemetry.addLine(e.name());
        telemetry.update();

        //I added what I think would be proper points for mechanisms to do what they need to do, but I commented them out just for now
        if (e == DetectColor.ColorLocation.LEFT || e == DetectColor.ColorLocation.UNDETECTED) {

            firstMove = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(frwDistance3)
                    .splineTo(new Vector2d(splineToLinear1X, splineToLinear1Y), Math.toRadians(spline1deg))
                    .build();

        } else if (e == DetectColor.ColorLocation.CENTER) {
            firstMove = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(frwDistance1)
                    .build();
        } else if (e == DetectColor.ColorLocation.RIGHT) {
            firstMove = drive.trajectorySequenceBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(linetoLinear1X, linetoLinear1Y, Math.toRadians(lineToLinear1Heading)))
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
        }

    }

}
