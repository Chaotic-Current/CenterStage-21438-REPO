package org.firstinspires.ftc.teamcode.Autos.AutoTests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Mechanisms.LowPass;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous
@Config
public class KalmanFilterTest extends LinearOpMode {
    public static double offset = 3.8;
    private double x,y;
    private Pose2d pastVelocity = new Pose2d();
    Rev2mDistanceSensor left, right;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private double angle;
    private double correction;

    private LowPass lowP = new LowPass(0,0.5);
    private ElapsedTime timer = new ElapsedTime();
    private IMU imu;



    @Override
    public void runOpMode() {
        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);
        bot.getEncoder().setCamTelemetry(telemetry);
        /*
        bot.getEncoder().setFrontBoardInfo(new ArrayList<Double>(Arrays.asList(

        ))); */
        left = hardwareMap.get(Rev2mDistanceSensor.class, "left"); // apparently i2c bus 0 is alr in use by imu so don't plug anything in there
        right = hardwareMap.get(Rev2mDistanceSensor.class, "right");
        double angle = 0;

           /*
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters); */


        TrajectorySequence traj1 = bot.trajectorySequenceBuilder(new Pose2d(0,0,0))
               // .lineToLinearHeading(new Pose2d(25, 0, Math.toRadians(-90)))
               // .splineToConstantHeading(new Vector2d(24,-24))
                //.splineToConstantHeading(new Vector2d(24,-24),Math.toRadians(0))
                //.forward(6)
                //.waitSeconds(2)
               // .forward(8)
                .setConstraints(SampleMecanumDrive.getVelocityConstraint(15, 10, 11.5) , SampleMecanumDrive.getAccelerationConstraint(7))
                .lineToSplineHeading(new Pose2d(35,-24,Math.toRadians(0)))
                .resetConstraints()
                .waitSeconds(28)
                .build();

        waitForStart();

        bot.followTrajectorySequenceAsync(traj1);
        while (opModeIsActive() && !isStopRequested()) { // !slides.atTarget && for feedforward, // slides.update(); in loop
            timer.reset();
            bot.update();
            telemetry.addData("Kalman(perpendicular)",bot.getEncoder().getMultiSensorFuser().getX_k().get(0,0));
            telemetry.addData("perpendicular encoder",bot.getEncoder().getPerpendicularEncoderPos());
            telemetry.addData("Kalman Gain",bot.getEncoder().getMultiSensorFuser().getK_k().get(0,0));
            telemetry.addData("loopSpeed",lowP.execute(timer.seconds()));
            telemetry.update();
        }

        angle = Math.toDegrees(Math.atan((right.getDistance(DistanceUnit.INCH) - left.getDistance(DistanceUnit.INCH)) / 11.2) + Math.toRadians(offset));
        correction = Math.toDegrees(bot.getPoseEstimate().getHeading() + Math.toRadians(angle));

        /* cd C:\Users\Indra\AppData\Local\Android\Sdk
cd platform-tools
adb connect 192.168.43.1:5555
*/
        //Pose2d traj2StartPosewOffset = new Pose2d(traj1.end().getX()+depthCalculation(bot.getPoseEstimate().getX(),60,y),traj1.end().getY()-x,traj1.end().getHeading());
        //bot.setPoseEstimate(traj2StartPosewOffset);
        /*
        TrajectorySequence traj2 = bot.trajectorySequenceBuilder(bot.getPoseEstimate())
                //.turn(angle) bot.getPoseEstimate().getHeading() - angle
               // .setConstraints(SampleMecanumDrive.getVelocityConstraint(15, 10, 11.5) , SampleMecanumDrive.getAccelerationConstraint(7))
                //.lineTo(new Vector2d(40,-24))
                .lineToSplineHeading(new Pose2d(35,0,Math.toRadians(0)))
                //.resetConstraints()
                //.waitSeconds(8)
                //.lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(correction)))
                .build();

       bot.followTrajectorySequenceAsync(traj2); */
      // bot.update();

        while ( opModeIsActive() && bot.isBusy()) { // !slides.atTarget && for feedforward, // slides.update(); in loop
            bot.update();
            telemetry.addLine("angle (degrees): " + Math.toDegrees(angle));
            telemetry.addLine(" correction: " + correction);
            telemetry.addData("Kalman(perpendicular)",bot.getEncoder().getMultiSensorFuser().getX_k().get(0,0));
            telemetry.addData("perpendicular encoder",bot.getEncoder().getPerpendicularEncoderPos());
            telemetry.addData("Kalman Gain",bot.getEncoder().getMultiSensorFuser().getK_k());
            telemetry.update();
        }

    }

    public static double depthCalculation(double currentPose, double boardPosition, double camDepth){
        return (boardPosition-currentPose)-(camDepth);
    }


}
