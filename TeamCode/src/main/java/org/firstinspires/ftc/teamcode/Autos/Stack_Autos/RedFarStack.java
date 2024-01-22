package org.firstinspires.ftc.teamcode.Autos.Stack_Autos;

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

@Autonomous(name = "AAA redfarsideSTACK")
@SuppressWarnings("all")
public class RedFarStack extends LinearOpMode {
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
	public static double centerFrwDistance1 = 28.5;
	public static double centerBackwardsDistance1 = 12;
	public static double frwDistance2 = 5;
	public static double frwDistance3 = 12;
	public static double wait01 = 1;
	public static double wait02 = 1;
	public static double centerLineToLinear1X = 25, centerLineToLinear1Y = 14, centerLineToLinear1Heading = -92, wait1Center = 3;

	public static double centerSplineToLinearHeading1X = 1.5, centerSplineToLinearHeading1Y = -6, centerSplineToLinearHeading1EndTangent = -75, centerSplineToLinearHeading1Heading = -90;

	public static double centerLineTo1X = 1.5, centerLineTo1Y = -61;

	public static double centerSplineToLinearHeading2X = 27, centerSplineToLinearHeading2Y = -85, centerSplineToLinearHeading2EndTangent = -75, centerSplineToLinearHeading2Heading = -90;

	public static double leftSplineToLinearHeading2X = 31.5, leftSplineToLinearHeading2Y = -85, leftSplineToLinearHeading2EndTangent = -75, leftSplineToLinearHeading2Heading = -90;

	public static double rightSplineToLinearHeading2X = 22.5, rightSplineToLinearHeading2Y = -85, rightSplineToLinearHeading2EndTangent = -75, rightSplineToLinearHeading2Heading = -90;

	public static double centerLineToLinear2Y = 41;

	// LEFT
	public static double leftLineToLinear2X = 20, leftLineToLinear2Y = 33.5, leftLineToLinear2Heading = 90, wait1Left = 3, wait2Left = 1;
	public static double leftLinetoLinear3Y = 40.5;
	public static double leftLinetoLinear1X = 26, leftLinetoLinear1Y = 7.5, leftLineToLinear1Heading = 30;
	public static double leftBackDist = 10;

	// RIGHT
	public static double rightSpline1deg = -75;
	public static double rightLineToLinear2deg = 90;
	public static double rightSplineTo1X = 26, rightSplineTo1Y = -2, splineToLinear1Heading = -80;
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

	public void cameraInit(){
		int width = 160;


		detector = new DetectColor(width, telemetry, new Scalar(10,255,255),new Scalar(2,100,100),true);

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
		clawMech.open();
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
		if (e == DetectColor.ColorLocation.RIGHT  || e == DetectColor.ColorLocation.UNDETECTED) {
			// aprilTagPipeline.setTargetTag(3);
			autoTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
					//.forward(frwDistance3)
					//.splineTo(new Vector2d(rightSplineTo1X, rightSplineTo1Y), Math.toRadians(rightSpline1deg))
					//.back(7)
					.lineToLinearHeading(new Pose2d(25, -14, Math.toRadians(-45)))
					.lineToLinearHeading(new Pose2d(centerLineToLinear1X, centerLineToLinear1Y, Math.toRadians(centerLineToLinear1Heading)))
					.splineToLinearHeading(new Pose2d(centerSplineToLinearHeading1X,centerSplineToLinearHeading1Y, Math.toRadians(centerSplineToLinearHeading1Heading)), Math.toRadians(centerSplineToLinearHeading1EndTangent))
					.lineTo(new Vector2d(centerLineTo1X, centerLineTo1Y))
					.splineToLinearHeading(new Pose2d(rightSplineToLinearHeading2X, rightSplineToLinearHeading2Y, Math.toRadians(rightSplineToLinearHeading2Heading)), Math.toRadians(rightSplineToLinearHeading2EndTangent))
					.waitSeconds(1)
					.forward(8)
					.build();

		} else if (e == DetectColor.ColorLocation.CENTER) {
			tagUse = 2;
			autoTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
					.lineToLinearHeading(new Pose2d(31.5, -6, Math.toRadians(-45)))
					//.forward(centerFrwDistance1)
					//.back(7)
					.UNSTABLE_addTemporalMarkerOffset(0, () -> {
						intake.start();
					})
					.lineToLinearHeading(new Pose2d(centerLineToLinear1X, centerLineToLinear1Y, Math.toRadians(centerLineToLinear1Heading)))

					.UNSTABLE_addTemporalMarkerOffset(-0.15, () -> {
						intake.AutoIntakeServoPositionStage1();
					})
					.UNSTABLE_addTemporalMarkerOffset(.25, () -> {
						//intake.AutoIntakeServoPositionStage2();
					})
					.UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
						clawMech.close();
					})
					.UNSTABLE_addTemporalMarkerOffset(2, () -> {
						intake.reverse();
					})
					.UNSTABLE_addTemporalMarkerOffset(2,()->{
						intake.stop();
					})

					.waitSeconds(2)
					.splineToLinearHeading(new Pose2d(centerSplineToLinearHeading1X,centerSplineToLinearHeading1Y, Math.toRadians(centerSplineToLinearHeading1Heading)), Math.toRadians(centerSplineToLinearHeading1EndTangent))
					.lineTo(new Vector2d(centerLineTo1X, centerLineTo1Y))
					.splineToLinearHeading(new Pose2d(centerSplineToLinearHeading2X, centerSplineToLinearHeading2Y, Math.toRadians(centerSplineToLinearHeading2Heading)), Math.toRadians(centerSplineToLinearHeading2EndTangent))
					.waitSeconds(1)
					.forward(8)
					.build();

		} else if (e == DetectColor.ColorLocation.LEFT) {
			autoTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
					//.lineToLinearHeading(new Pose2d(leftLinetoLinear1X, leftLinetoLinear1Y, Math.toRadians(leftLineToLinear1Heading)))
					.lineToLinearHeading(new Pose2d(27, -2.5, Math.toRadians(45)))
					//.back(7)
					.lineToLinearHeading(new Pose2d(centerLineToLinear1X, centerLineToLinear1Y, Math.toRadians(centerLineToLinear1Heading)))
					.splineToLinearHeading(new Pose2d(centerSplineToLinearHeading1X,centerSplineToLinearHeading1Y, Math.toRadians(centerSplineToLinearHeading1Heading)), Math.toRadians(centerSplineToLinearHeading1EndTangent))
					.lineTo(new Vector2d(centerLineTo1X, centerLineTo1Y))
					.splineToLinearHeading(new Pose2d(leftSplineToLinearHeading2X, leftSplineToLinearHeading2Y, Math.toRadians(leftSplineToLinearHeading2Heading)), Math.toRadians(leftSplineToLinearHeading2EndTangent))
					.waitSeconds(1)
					.forward(8)
					.build();

		} else {
			telemetry.addLine("ruh-roh (´▽`ʃ♡ƪ)");
		}
		waitForStart();

		drive.followTrajectorySequenceAsync(autoTrajectory);

		while (opModeIsActive() && !isStopRequested()) {
			telemetry.addLine(intake.getServosPos());
			drive.update();
			telemetry.update();
		}
	}
}
