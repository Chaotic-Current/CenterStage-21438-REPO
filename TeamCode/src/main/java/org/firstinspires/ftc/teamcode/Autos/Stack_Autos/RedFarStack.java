package org.firstinspires.ftc.teamcode.Autos.Stack_Autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
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
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous(name = "AAA redfarsideSTACK")
@Config
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

	private AprilTagProcessor aprilTagBack;

	private VisionPortal visionPortalBack;
	private int tagUse;
	public  double errorX;
	public  double errorY;
	public static double errorYaw;

	// CENTER
	public static double centerFrwDistance1 = 28.5;
	public static double centerBackwardsDistance1 = 12;
	public static double frwDistance2 = 5;
	public static double frwDistance3 = 12;
	public static double wait01 = 1;
	public static double wait02 = 1;
	public static double centerLineToLinear1X = 25.5, centerLineToLinear1Y = 13, centerLineToLinear1Heading = -92, wait1Center = 3;

	public static double centerSplineToLinearHeading1X = 1.5, centerSplineToLinearHeading1Y = -6, centerSplineToLinearHeading1EndTangent = -75, centerSplineToLinearHeading1Heading = -90;

	public static double centerLineTo1X = 1.5, centerLineTo1Y = -61;

	public static double centerSplineToLinearHeading2X = 27, centerSplineToLinearHeading2Y = -83, centerSplineToLinearHeading2EndTangent = -75, centerSplineToLinearHeading2Heading = -90;

	public static double leftSplineToLinearHeading2X = 31.5, leftSplineToLinearHeading2Y = -83, leftSplineToLinearHeading2EndTangent = -75, leftSplineToLinearHeading2Heading = -90;

	public static double rightSplineToLinearHeading2X = 22.5, rightSplineToLinearHeading2Y = -83, rightSplineToLinearHeading2EndTangent = -75, rightSplineToLinearHeading2Heading = -90;

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

	public static double yReduction = 13.45;

	public static double waitAtStack = 2;

	public static double yreductionBack = 12;

	public static double xreductionBack = 5.3;

	public static double tagUseBack = 10;

	TrajectorySequence autoTrajectory;

	private void initAprilTag() {

		// Create the AprilTag processor.
		aprilTag = new AprilTagProcessor.Builder()
				//.setDrawAxes(false)
				//.setDrawCubeProjection(false)
				//.setDrawTagOutline(true)
				//.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
				//.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
				//.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

				// == CAMERA CALIBRATION ==
				// If you do not manually specify calibration parameters, the SDK will attempt
				// to load a predefined calibration for your camera.
				//.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

				// ... these parameters are fx, fy, cx, cy.

				.build();

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
		//builder.enableCameraMonitoring(true);

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

	}   // end method initAprilTag()

	private void initAprilTagBack() {

		// Create the AprilTag processor.
		aprilTagBack = new AprilTagProcessor.Builder()
				//.setDrawAxes(false)
				//.setDrawCubeProjection(false)
				//.setDrawTagOutline(true)
				//.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
				//.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
				//.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

				// == CAMERA CALIBRATION ==
				// If you do not manually specify calibration parameters, the SDK will attempt
				// to load a predefined calibration for your camera.
				//.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

				// ... these parameters are fx, fy, cx, cy.

				.build();

		// Create the vision portal by using a builder.
		VisionPortal.Builder builder = new VisionPortal.Builder();

		// Set the camera (webcam vs. built-in RC phone camera).
		if (USE_WEBCAM) {
			builder.setCamera(hardwareMap.get(WebcamName.class, "BackCam"));
		} else {
			builder.setCamera(BuiltinCameraDirection.BACK);
		}

		// Choose a camera resolution. Not all cameras support all resolutions.
		//builder.setCameraResolution(new Size(640, 480));

		// Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
		//builder.enableCameraMonitoring(true);

		// Set the stream format; MJPEG uses less bandwidth than default YUY2.
		//builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

		// Choose whether or not LiveView stops if no processors are enabled.
		// If set "true", monitor shows solid orange screen if no processors enabled.
		// If set "false", monitor shows camera view without annotations.
		//builder.setAutoStopLiveView(false);

		// Set and enable the processor.
		builder.addProcessor(aprilTagBack);

		// Build the Vision Portal, using the above settings.
		visionPortalBack = builder.build();

		// Disable or re-enable the aprilTag processor at any time.
		//visionPortal.setProcessorEnabled(aprilTag, true);

	}   // end method initAprilTag()


	/**
	 * Add telemetry about AprilTag detections.
	 */
	private void telemetryAprilTag() {

		List<AprilTagDetection> currentDetections = aprilTag.getDetections();
		telemetry.addData("# AprilTags Detected", currentDetections.size());

		// Step through the list of detections and display info for each one.
		for (AprilTagDetection detection : currentDetections) {
			if (detection.id == tagUse) {
				errorX = -detection.ftcPose.x*0.9;
				errorY = -(detection.ftcPose.y-yReduction);
				telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
				telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
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

	private void telemetryAprilTagBack() {

		List<AprilTagDetection> currentDetections = aprilTagBack.getDetections();
		telemetry.addData("# AprilTags Detected", currentDetections.size());

		// Step through the list of detections and display info for each one.
		for (AprilTagDetection detection : currentDetections) {
			if (detection.id == tagUseBack) {
				errorX = -(detection.ftcPose.x + xreductionBack);
				errorY = (detection.ftcPose.y - yreductionBack);
				telemetry.addLine("x " + (detection.ftcPose.x - xreductionBack) + "y " + (detection.ftcPose.y - yreductionBack));
				telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
				telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
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

	public boolean readyToScan(){
		if(Math.abs(errorY) != 0 && Math.abs(errorY) < 5){
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
			if (detection.id == tagUse) {
				int multiplier = detection.ftcPose.x >= 0 ? 1 : -1;
				errorX = -((Math.abs(detection.ftcPose.x))*multiplier);
				errorY = (detection.ftcPose.y- yReduction);
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
		intake = new IntakeMech(hardwareMap, telemetry);
		wrist = hardwareMap.get(Servo.class, "WRIST");
		wrist.setPosition(0.5);
		clawMech.open();
		cameraInit();
	}

	@Override
	public void runOpMode() throws InterruptedException {
		errorX = 0;
		errorY = 0;

		initialize();

		drive.setPoseEstimate(new Pose2d(0,-3,0));

		DetectColor.ColorLocation e = detector.getLocate();

		while (!isStarted()) {
			e = detector.getLocate();


			telemetry.update();
		}
		frontCam.stopStreaming();
		frontCam.closeCameraDevice();
		initAprilTag();

//|| e == DetectColor.ColorLocation.UNDETECTED
		if (e == DetectColor.ColorLocation.RIGHT  || e == DetectColor.ColorLocation.UNDETECTED) {
			// aprilTagPipeline.setTargetTag(3);
			tagUse = 6;
			autoTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
					.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
						clawMech.open();
						intake.setServosUp();
						intake.setIsReduced(true);
					})
					.lineToLinearHeading(new Pose2d(25, -14, Math.toRadians(-45)))
					.lineToLinearHeading(new Pose2d(centerLineTo1X, centerLineToLinear1Y-6,Math.toRadians(centerLineToLinear1Heading)))
					.waitSeconds(0.5)
					.UNSTABLE_addTemporalMarkerOffset(0,()->{
						intake.setIsReduced(false);
					})
					.lineToLinearHeading(new Pose2d(centerLineToLinear1X, centerLineToLinear1Y, Math.toRadians(centerLineToLinear1Heading)))
					.waitSeconds(waitAtStack)
					.UNSTABLE_addTemporalMarkerOffset(0,()->{
						intake.reverse();
					})
					.UNSTABLE_addDisplacementMarkerOffset(2.5,()->{
						intake.stop();
						clawMech.close();
						intake.AutoIntakeServoPositionStage1();
					})
					.splineToLinearHeading(new Pose2d(centerSplineToLinearHeading1X,centerSplineToLinearHeading1Y, Math.toRadians(centerSplineToLinearHeading1Heading)), Math.toRadians(centerSplineToLinearHeading1EndTangent))
					.lineTo(new Vector2d(centerLineTo1X, centerLineTo1Y))
					.UNSTABLE_addTemporalMarkerOffset(0.75,()->{
						slide.setCustom(1200);
					})
					.UNSTABLE_addTemporalMarkerOffset(1.25,()->{
						arm.setExtake();
					})
					.splineToLinearHeading(new Pose2d(rightSplineToLinearHeading2X-4, rightSplineToLinearHeading2Y, Math.toRadians(rightSplineToLinearHeading2Heading)), Math.toRadians(rightSplineToLinearHeading2EndTangent))
					.waitSeconds(1.5)
					.lineToLinearHeading(new Pose2d(rightSplineToLinearHeading2X-4,rightSplineToLinearHeading2Y-9.5, Math.toRadians(rightSplineToLinearHeading2Heading)))
					.UNSTABLE_addTemporalMarkerOffset(0,()->{
						clawMech.halfOpen();
					})
					.waitSeconds(0.5)
					.UNSTABLE_addTemporalMarkerOffset(0.2,()->{
						slide.setCustom(1470);
					})
					.back(1.5)
					.waitSeconds(0.5)
					.forward(1.5)
					.UNSTABLE_addTemporalMarkerOffset(0.5,()->{
						clawMech.open();
					})
					.UNSTABLE_addTemporalMarkerOffset(1.25,()->{
						clawMech.close();
					})
					.waitSeconds(1.5)
					.UNSTABLE_addTemporalMarkerOffset(0.5,()->{
						arm.setIntake();
					})
					.UNSTABLE_addTemporalMarkerOffset(1.5,()->{
						slide.setIntakeOrGround();
					})
					.back(10)
					.lineToLinearHeading(new Pose2d(0,-85,Math.toRadians(-90)))
					.forward(12)
					.build();

		} else if (e == DetectColor.ColorLocation.CENTER) {
			tagUse = 5;
			autoTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
					.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
						clawMech.open();
						intake.setServosUp();
						intake.setIsReduced(true);
					})
					.lineToLinearHeading(new Pose2d(31.5, -6, Math.toRadians(-45)))
					.lineToLinearHeading(new Pose2d(centerLineTo1X, centerLineToLinear1Y-6,Math.toRadians(centerLineToLinear1Heading)))
					.waitSeconds(0.5)
					.UNSTABLE_addTemporalMarkerOffset(0,()->{
						intake.setIsReduced(false);
					})
					.lineToLinearHeading(new Pose2d(centerLineToLinear1X, centerLineToLinear1Y, Math.toRadians(centerLineToLinear1Heading)))
					.waitSeconds(waitAtStack)
					.UNSTABLE_addTemporalMarkerOffset(0,()->{
						intake.reverse();
					})
					.UNSTABLE_addDisplacementMarkerOffset(2.5,()->{
						intake.stop();
						clawMech.close();
						intake.AutoIntakeServoPositionStage1();
					})
					.splineToLinearHeading(new Pose2d(centerSplineToLinearHeading1X,centerSplineToLinearHeading1Y, Math.toRadians(centerSplineToLinearHeading1Heading)), Math.toRadians(centerSplineToLinearHeading1EndTangent))
					.lineTo(new Vector2d(centerLineTo1X, centerLineTo1Y))
					.UNSTABLE_addTemporalMarkerOffset(0.75,()->{
						slide.setCustom(1200);
					})
					.UNSTABLE_addTemporalMarkerOffset(1.25,()->{
						arm.setExtake();
					})
					.splineToLinearHeading(new Pose2d(centerSplineToLinearHeading2X-2, centerSplineToLinearHeading2Y, Math.toRadians(centerSplineToLinearHeading2Heading)), Math.toRadians(centerSplineToLinearHeading2EndTangent))
					.waitSeconds(1.5)
					.lineToLinearHeading(new Pose2d(centerSplineToLinearHeading2X-2,centerSplineToLinearHeading2Y-10,Math.toRadians(centerSplineToLinearHeading2Heading)))
					.UNSTABLE_addTemporalMarkerOffset(0,()->{
						clawMech.halfOpen();
					})
					.waitSeconds(0.5)
					.UNSTABLE_addTemporalMarkerOffset(0.2,()->{
						slide.setCustom(1470);
					})
					.back(1.5)
					.strafeRight(10)
					.waitSeconds(0.5)
					.forward(1.5)
					.UNSTABLE_addTemporalMarkerOffset(0.5,()->{
						clawMech.open();
					})
					.UNSTABLE_addTemporalMarkerOffset(1.25,()->{
						clawMech.close();
					})
					.waitSeconds(1.5)
					.UNSTABLE_addTemporalMarkerOffset(0.5,()->{
						arm.setIntake();
					})
					.UNSTABLE_addTemporalMarkerOffset(1.5,()->{
						slide.setIntakeOrGround();
					})
					.back(10)
					.lineToLinearHeading(new Pose2d(0,-85,Math.toRadians(-90)))
					.forward(12)
					.build();

		} else if (e == DetectColor.ColorLocation.LEFT) {
			tagUse = 4;
			autoTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
					//.lineToLinearHeading(new Pose2d(leftLinetoLinear1X, leftLinetoLinear1Y, Math.toRadians(leftLineToLinear1Heading)))
					.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
						clawMech.open();
						intake.setServosUp();
						intake.setIsReduced(true);
					})
					.lineToLinearHeading(new Pose2d(27, -2.5, Math.toRadians(45)))
					.lineToLinearHeading(new Pose2d(centerLineTo1X, centerLineToLinear1Y-6,Math.toRadians(centerLineToLinear1Heading)))
					.waitSeconds(0.5)
					.UNSTABLE_addTemporalMarkerOffset(0,()->{
						intake.setIsReduced(false);
					})
					.lineToLinearHeading(new Pose2d(centerLineToLinear1X, centerLineToLinear1Y, Math.toRadians(centerLineToLinear1Heading)))
					.waitSeconds(waitAtStack)
					.UNSTABLE_addTemporalMarkerOffset(0,()->{
						intake.reverse();
					})
					.UNSTABLE_addDisplacementMarkerOffset(2.5,()->{
						intake.stop();
						clawMech.close();
						intake.AutoIntakeServoPositionStage1();
					})
					.splineToLinearHeading(new Pose2d(centerSplineToLinearHeading1X,centerSplineToLinearHeading1Y, Math.toRadians(centerSplineToLinearHeading1Heading)), Math.toRadians(centerSplineToLinearHeading1EndTangent))
					.lineTo(new Vector2d(centerLineTo1X, centerLineTo1Y))
					.UNSTABLE_addTemporalMarkerOffset(0.75,()->{
						slide.setCustom(1200);
					})
					.UNSTABLE_addTemporalMarkerOffset(1.25,()->{
						arm.setExtake();
					})
					.splineToLinearHeading(new Pose2d(leftSplineToLinearHeading2X-4, leftSplineToLinearHeading2Y, Math.toRadians(leftSplineToLinearHeading2Heading)), Math.toRadians(leftSplineToLinearHeading2EndTangent))

					.waitSeconds(1.5)
					.lineToLinearHeading(new Pose2d(leftSplineToLinearHeading2X-4, leftSplineToLinearHeading2Y-10, Math.toRadians(leftSplineToLinearHeading2Heading)))
					.UNSTABLE_addTemporalMarkerOffset(0,()->{
						clawMech.halfOpen();
					})
					.waitSeconds(0.5)
					.UNSTABLE_addTemporalMarkerOffset(0.2,()->{
						slide.setCustom(1470);
					})
					.back(1.5)
					.strafeRight(10)
					.waitSeconds(0.5)
					.forward(1.5)
					.UNSTABLE_addTemporalMarkerOffset(0.5,()->{
						clawMech.open();
					})
					.UNSTABLE_addTemporalMarkerOffset(1.25,()->{
						clawMech.close();
					})
					.waitSeconds(1.5)
					.UNSTABLE_addTemporalMarkerOffset(0.5,()->{
						arm.setIntake();
					})
					.UNSTABLE_addTemporalMarkerOffset(1.5,()->{
						slide.setIntakeOrGround();
					})
					.back(10)
					.lineToLinearHeading(new Pose2d(0,-85,Math.toRadians(-90)))
					.forward(12)
					.build();

		} else {
			telemetry.addLine("ruh-roh (´▽`ʃ♡ƪ)");
		}
		waitForStart();

		drive.followTrajectorySequenceAsync(autoTrajectory);
		//hello
		boolean hasRanOnce = false;
		while (opModeIsActive() && !isStopRequested()) {
			telemetry.addLine(e.name());
			telemetry.addLine(errorX + "-x, " + errorY + "-y");
			telemetry.addLine("Encoder Pos: " + drive.getEncoder().getWheelPositions());
			boolean readyToScan = readyToScan();
			telemetry.addLine("Is ready to scan "+ readyToScan);

			if(readyToScan && !hasRanOnce && drive.getEncoder().getWheelVelocitySum() == 0){
				telemetry.addLine("Offsets added");
				drive.getEncoder().setErrorX(errorX);
				drive.getEncoder().setErrorY(errorY);
				hasRanOnce = true;

			}

			telemetryAprilTag();
			telemetry.addLine(""+drive.getPoseEstimate());
			telemetry.addLine(drive.getEncoder().getErrorX() + "-x, " + drive.getEncoder().getErrorY() + "-y");
			telemetry.addLine(intake.getServosPos());
			drive.update();
			slide.update();
			intake.update(drive);
			telemetry.update();
		}
	}
}
