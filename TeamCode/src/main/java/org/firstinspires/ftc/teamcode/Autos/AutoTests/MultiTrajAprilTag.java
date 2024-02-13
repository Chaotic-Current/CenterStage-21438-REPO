package org.firstinspires.ftc.teamcode.Autos.AutoTests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
@Config
public class MultiTrajAprilTag extends LinearOpMode {
    public static double offset = 3.8;
    private double x,y;
    Rev2mDistanceSensor left, right;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private double angle;
    private double correction;
    private IMU imu;

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder().build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "WebcamFront"));

        builder.addProcessor(aprilTag);


    }   // end method initAprilTag()

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id > -1) {
                x = -detection.ftcPose.x;
                y = detection.ftcPose.y - 10;
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y-10, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry


    }   // end method telemetryAprilTag()

    @Override
    public void runOpMode() {
        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);
        left = hardwareMap.get(Rev2mDistanceSensor.class, "left"); // apparently i2c bus 0 is alr in use by imu so don't plug anything in there
        right = hardwareMap.get(Rev2mDistanceSensor.class, "right");
        double angle = 0;
        initAprilTag();
           /*
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters); */

        while (!isStarted()) {
            angle = Math.atan((right.getDistance(DistanceUnit.INCH) - left.getDistance(DistanceUnit.INCH)) / 11.2);
            telemetry.addLine("Initial left distance (inches): " + left.getDistance(DistanceUnit.INCH));
            telemetry.addLine("Initial right distance (inches): " + right.getDistance(DistanceUnit.INCH));
            telemetry.addLine("Initial angle (degrees): " + Math.toDegrees(angle));
            telemetry.update();
        }

        TrajectorySequence traj1 = bot.trajectorySequenceBuilder(new Pose2d(0,0,0))
                .lineToLinearHeading(new Pose2d(10, 0, 0))
                .waitSeconds(0.5)
                .build();

        bot.followTrajectorySequenceAsync(traj1);
        while (bot.isBusy() && opModeIsActive() && !isStopRequested()) { // !slides.atTarget && for feedforward, // slides.update(); in loop
            bot.update();
            telemetryAprilTag();
        }

        angle = Math.toDegrees(Math.atan((right.getDistance(DistanceUnit.INCH) - left.getDistance(DistanceUnit.INCH)) / 11.2) + Math.toRadians(offset));
        correction = Math.toDegrees(bot.getPoseEstimate().getHeading() + Math.toRadians(angle));

        Pose2d traj2StartPose = new Pose2d(traj1.end().getX()+x,traj1.end().getY()+y,traj1.end().getHeading());
        TrajectorySequence traj2 = bot.trajectorySequenceBuilder(traj2StartPose)
                //.turn(angle) bot.getPoseEstimate().getHeading() - angle
                .lineTo(new Vector2d(15,0))
                .waitSeconds(1.0)
                //.lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(correction)))
                .build();

       // bot.followTrajectorySequenceAsync(traj2);

        while (bot.isBusy() && opModeIsActive()) { // !slides.atTarget && for feedforward, // slides.update(); in loop
            bot.update();
            telemetry.addLine("angle (degrees): " + Math.toDegrees(angle));
            telemetry.addLine(" correction: " + correction);
            telemetry.update();
        }

    }

}
