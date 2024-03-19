package org.firstinspires.ftc.teamcode.Autos.AutoTests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
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
    private Pose2d pastVelocity = new Pose2d();
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

        visionPortal = builder.build();


    }   // end method initAprilTag()

    private void aprilTagUpdate() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 5) {
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
            aprilTagUpdate();
            telemetry.update();
        }

        TrajectorySequence traj1 = bot.trajectorySequenceBuilder(new Pose2d(0,0,0))
               // .lineToLinearHeading(new Pose2d(25, 0, Math.toRadians(-90)))
               // .splineToConstantHeading(new Vector2d(24,-24))
               // .splineToConstantHeading(new Vector2d(24,-24),Math.toRadians(0))
                //.forward(6)
                //.waitSeconds(2)
                .forward(16)
                .build();

        bot.followTrajectorySequenceAsync(traj1);
        while (bot.isBusy() && opModeIsActive() && !isStopRequested()) { // !slides.atTarget && for feedforward, // slides.update(); in loop
            bot.update();
            aprilTagUpdate();
            telemetry.update();
        }

        angle = Math.toDegrees(Math.atan((right.getDistance(DistanceUnit.INCH) - left.getDistance(DistanceUnit.INCH)) / 11.2) + Math.toRadians(offset));
        correction = Math.toDegrees(bot.getPoseEstimate().getHeading() + Math.toRadians(angle));

        /* cd C:\Users\Indra\AppData\Local\Android\Sdk
cd platform-tools
adb connect 192.168.43.1:5555
*/
        Pose2d traj2StartPosewOffset = new Pose2d(traj1.end().getX()+depthCalculation(bot.getPoseEstimate().getX(),60,y),traj1.end().getY()-x,traj1.end().getHeading());
        bot.setPoseEstimate(traj2StartPosewOffset);
        TrajectorySequence traj2 = bot.trajectorySequenceBuilder(traj2StartPosewOffset)
                //.turn(angle) bot.getPoseEstimate().getHeading() - angle
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(15, 10, 11.5) , SampleMecanumDrive.getAccelerationConstraint(7))
                //.lineTo(new Vector2d(40,-24))
                .lineToSplineHeading(new Pose2d(60,0,Math.toRadians(0)))
                //.resetConstraints()
                .waitSeconds(1.0)
                //.lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(correction)))
                .build();

       bot.followTrajectorySequenceAsync(traj2);
       bot.update();

        while (bot.isBusy() && opModeIsActive()) { // !slides.atTarget && for feedforward, // slides.update(); in loop
            bot.update();
            telemetry.addLine("angle (degrees): " + Math.toDegrees(angle));
            telemetry.addLine(" correction: " + correction);
            telemetry.update();
        }

    }

    public static double depthCalculation(double currentPose, double boardPosition, double camDepth){
        return (boardPosition-currentPose)-(camDepth);
    }


}
