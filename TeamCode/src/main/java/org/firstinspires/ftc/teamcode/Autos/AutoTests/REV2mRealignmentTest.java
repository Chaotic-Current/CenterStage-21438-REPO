package org.firstinspires.ftc.teamcode.Autos.AutoTests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class REV2mRealignmentTest extends LinearOpMode {
    Rev2mDistanceSensor left, right;
    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.get(Rev2mDistanceSensor.class, "left"); // apparently i2c bus 0 is alr in use by imu so don't plug anything in there
        right = hardwareMap.get(Rev2mDistanceSensor.class, "right");
        double angle = 0;

        while (!isStarted()){
            angle = Math.atan((right.getDistance(DistanceUnit.INCH)-left.getDistance(DistanceUnit.INCH)) / 11.2); // Math.atan() --> radians
            //angle /= 1.65;
            telemetry.addLine("Left distance (inches): " + left.getDistance(DistanceUnit.INCH));
            telemetry.addLine("Right distance (inches): " + right.getDistance(DistanceUnit.INCH));
            telemetry.addLine("Angle (degrees): " + Math.toDegrees(angle));
            telemetry.update();
        }

        waitForStart();

        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence move = bot.trajectorySequenceBuilder(new Pose2d())
                //.lineToLinearHeading(new Pose2d(0,1, angle))
                .turn(angle) // angle is already in radians
                .build();

        bot.followTrajectorySequence(move);
    }
}
