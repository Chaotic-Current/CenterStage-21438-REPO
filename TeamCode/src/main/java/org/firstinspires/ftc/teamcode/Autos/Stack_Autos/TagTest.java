package org.firstinspires.ftc.teamcode.Autos.Stack_Autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.MechanismTests.AprilTagCam;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
@Config
public class TagTest extends LinearOpMode {
    public static double offset = 3.8;
    private double x,y;
    private Pose2d pastVelocity = new Pose2d();
    Rev2mDistanceSensor left, right;
    private AprilTagProcessor aprilTag;

    private AprilTagCam cam;
    private Intake intake;

    private VisionPortal visionPortal;
    private double angle;
    private double correction;

    private TrajectorySequence traj2;
    private Pose2d  traj2StartPosewOffset;
    private IMU imu;

    private void initAprilTag() {

        // Create the AprilTag processor.
        cam = new AprilTagCam(hardwareMap,"WebcamFront",5);
        cam.setTelemetry(telemetry);


    }   // end method initAprilTag()



    @Override
    public void runOpMode() {
        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);
        left = hardwareMap.get(Rev2mDistanceSensor.class, "left"); // apparently i2c bus 0 is alr in use by imu so don't plug anything in there
        right = hardwareMap.get(Rev2mDistanceSensor.class, "right");
        double angle = 0;
        int propPos = 1;
        initAprilTag();
       // intake = new Intake(hardwareMap,telemetry);


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
           cam.aprilTagUpdate();
            telemetry.update();
        }
        Pose2d blueStart = new Pose2d(14, 61, Math.toRadians(270));
        bot.setPoseEstimate(blueStart);
        TrajectorySequence firstBlueTraj = bot.trajectorySequenceBuilder(blueStart)
                .lineToLinearHeading(new Pose2d(14, 50, Math.toRadians(270)))
                .splineToSplineHeading(new Pose2d(18, 35, Math.toRadians(315)), Math.toRadians(315))
                .setReversed(true)
                .back(20) //tune this distance frfr
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // outake.extendOutake(500);
                    //extend outtake
                })
                .lineToSplineHeading(new Pose2d(36, 40, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(60, 31), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //outake.openClawB();
                    //open claw
                })
                .waitSeconds(2)
                //relocalize
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //outake.resetOutake();
                    //retract outtake
                    cam.setTargetID(4);
                })
                .back(7)
                .build();
        TrajectorySequence secondBlueTraj = bot.trajectorySequenceBuilder(blueStart)
                .lineToLinearHeading(new Pose2d(14, 32, Math.toRadians(270)))
                .setReversed(true)
                .back(20) //tune this distance frfr
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //extend outtake
                    // outake.extendOutake(500);
                })
                .lineToSplineHeading(new Pose2d(36, 40, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(45, 31), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //open claw
                    // outake.openClawB();
                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    // outake.resetOutake();
                    //retract outtake
                    cam.setTargetID(5);
                })
                .back(7)
                .build();
        TrajectorySequence thirdBlueTraj = bot.trajectorySequenceBuilder(blueStart)
                .lineToLinearHeading(new Pose2d(14, 40, Math.toRadians(270)))
                //.splineToSplineHeading(some position) for spike
                .setReversed(true)
                .back(10) //tune this distance frfr
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //extend outtake
                })
                .splineToConstantHeading(new Vector2d(19, 40), Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(36, 40, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(60, 31), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //open claw
                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    // outake.resetOutake();
                    //retract outtake
                    cam.setTargetID(6);
                })
                .back(7)
                .build();

        TrajectorySequence[] list = {firstBlueTraj,secondBlueTraj,thirdBlueTraj};

        switch(propPos){
            case 1:
                bot.followTrajectorySequenceAsync(firstBlueTraj);
                break;
            case 2:
                bot.followTrajectorySequenceAsync(secondBlueTraj);
                break;
            case 3:
                bot.followTrajectorySequenceAsync(thirdBlueTraj);
                break;
        }

       // bot.followTrajectorySequenceAsync(traj1);
        while (bot.isBusy() && opModeIsActive() && !isStopRequested()) { // !slides.atTarget && for feedforward, // slides.update(); in loop
            bot.update();
            cam.aprilTagUpdate();
            telemetry.update();
        }

        angle = Math.toDegrees(Math.atan((right.getDistance(DistanceUnit.INCH) - left.getDistance(DistanceUnit.INCH)) / 11.2) + Math.toRadians(offset));
        correction = Math.toDegrees(bot.getPoseEstimate().getHeading() + Math.toRadians(angle));

        /* cd C:\Users\Indra\AppData\Local\Android\Sdk
cd platform-tools
adb connect 192.168.43.1:5555
*/
        traj2StartPosewOffset = new Pose2d(list[propPos-1].end().getX(),list[propPos-1].end().getY()-x,bot.getPoseEstimate().getHeading());
        bot.setPoseEstimate(traj2StartPosewOffset);
        traj2 = bot.trajectorySequenceBuilder(traj2StartPosewOffset)
                .lineToLinearHeading(new Pose2d(40, 25, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(20,18), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-60, 18, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    //run intake
                    cam.setTargetID(6);
                    intake.setToIntake(0.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(1,()->{
                    intake.setToIntake(0.79);
                })
                .UNSTABLE_addTemporalMarkerOffset(3,()->{
                    intake.setToGround();
                })
                .waitSeconds(2)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(23, 18, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //extend outtake
                })
                .splineToConstantHeading(new Vector2d(42,31), Math.toRadians(45))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    //open claw
                })
                .waitSeconds(2)
                .back(7)
                .build();

       bot.followTrajectorySequenceAsync(traj2);

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
