package org.firstinspires.ftc.teamcode.Autos.AutoTests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Mechanisms.SlideMech;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous
@Disabled
public class AprilTagTest extends LinearOpMode {

    SampleMecanumDrive drive;

    private Pose2d startingPose = new Pose2d(0,0,0);

    double error = 2.0;

    double x;
    double y;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;


    void initialize() {
        initAprilTag();
        drive = new SampleMecanumDrive(hardwareMap);
    }

    private void telemetryAprilTag() {

        List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 2) {
                x = detection.ftcPose.x;
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
        telemetry.addLine(drive.getPoseEstimate().getX() + " " + drive.getPoseEstimate().getY() + " " + drive.getPoseEstimate().getHeading() );

    }   // end method telemetryAprilTag()


    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                .build();


        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "WebcamFront"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }


        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();



    }   // end method initAprilTag()


    @Override
    public void runOpMode() {
        SlideMech mech = new SlideMech(hardwareMap);

       AtomicBoolean t = new AtomicBoolean(false);
        initialize();

        TrajectorySequence move = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(3, 0, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    //telemetryAprilTag();
                    mech.setLowJunction();
                    t.set(true);
                })
                .waitSeconds(6)
                .lineToLinearHeading(new Pose2d(0,10,Math.toRadians(90)))
                .build();

        waitForStart();

        drive.followTrajectorySequenceAsync(move);

        while (opModeIsActive()) {
            telemetryAprilTag();
            mech.update();
            drive.update();
            if(Math.abs(drive.getPoseEstimate().getX() - 3) < 3 && Math.abs(drive.getPoseEstimate().getY() - startingPose.getY()) < 3 && t.get())
                drive.setPoseEstimate(new Pose2d(-x,-(y-13.8),Math.toRadians(90)));
                t.set(false);
            telemetry.update();
        }

    }
}
