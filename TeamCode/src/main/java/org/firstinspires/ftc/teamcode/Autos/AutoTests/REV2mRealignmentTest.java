package org.firstinspires.ftc.teamcode.Autos.AutoTests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Mechanisms.MechanismTests.REV2mArrayMech;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class REV2mRealignmentTest extends LinearOpMode {
    Rev2mDistanceSensor left, right;
    REV2mArrayMech mech;
    private volatile double angle;
    private IMU imu;

    public enum State{
        on, off
    }

    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.get(Rev2mDistanceSensor.class, "left"); // apparently i2c bus 0 is alr in use by imu so don't plug anything in there
        right = hardwareMap.get(Rev2mDistanceSensor.class, "right");
        double angle = 0;

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        while (!isStarted()){
            angle = Math.atan((right.getDistance(DistanceUnit.INCH)-left.getDistance(DistanceUnit.INCH)) / 11.2);
            telemetry.addLine("Left distance (inches): " + left.getDistance(DistanceUnit.INCH));
            telemetry.addLine("Right distance (inches): " + right.getDistance(DistanceUnit.INCH));
            telemetry.addLine("Angle (degrees): " + Math.toDegrees(angle));
            //telemetry.addData("IMU", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }

        waitForStart();

        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence move = bot.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(10, 0, Math.toRadians(-20)))
                .waitSeconds(1.5)
                .turn(angle) // we'll fix later
                .waitSeconds(1.5)
                //.lineToLinearHeading(new Pose2d(15, -10, Math.toRadians(0)))
                .build();

        bot.followTrajectorySequenceAsync(move);

        while (opModeIsActive() && !isStopRequested()) {
            bot.update();

        }
    }

}
