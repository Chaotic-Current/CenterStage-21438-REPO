package org.firstinspires.ftc.teamcode.Autos.AutoTests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class RuckusProgrammingTest extends LinearOpMode {
   Rev2mDistanceSensor left, right;
   private  double angle;
   private IMU imu;


   @Override
   public void runOpMode() {
      SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);
      left = hardwareMap.get(Rev2mDistanceSensor.class, "left"); // apparently i2c bus 0 is alr in use by imu so don't plug anything in there
      right = hardwareMap.get(Rev2mDistanceSensor.class, "right");
      double angle = 0;

      imu = hardwareMap.get(IMU.class, "imu");
      IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
              RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
              RevHubOrientationOnRobot.UsbFacingDirection.UP));
      imu.initialize(parameters);

      while (!isStarted()) {
         angle = Math.atan((right.getDistance(DistanceUnit.INCH) - left.getDistance(DistanceUnit.INCH)) / 11.2);
         telemetry.addLine("Initial left distance (inches): " + left.getDistance(DistanceUnit.INCH));
         telemetry.addLine("Initial right distance (inches): " + right.getDistance(DistanceUnit.INCH));
         telemetry.addLine("Initial angle (degrees): " + Math.toDegrees(angle));
         telemetry.update();
      }

      TrajectorySequence traj1 = bot.trajectorySequenceBuilder(new Pose2d())
              .lineToLinearHeading(new Pose2d(10, 0, Math.toRadians(-20))) // -20 used in setPoseEstimate
              .waitSeconds(1.5)
              .build();

      TrajectorySequence traj2 = bot.trajectorySequenceBuilder(new Pose2d())
              .lineToLinearHeading(new Pose2d(-20, 10, Math.toRadians(-40)))
              .waitSeconds(1.5)
              .build();

      bot.followTrajectorySequenceAsync(traj1);

      while(bot.isBusy() && opModeIsActive() && !isStopRequested()) { // !slides.atTarget && for feedforward, // slides.update(); in loop
         bot.update();
      }

      angle = Math.atan((right.getDistance(DistanceUnit.INCH) - left.getDistance(DistanceUnit.INCH)) / 11.2);
      bot.setPoseEstimate(new Pose2d(0,0, -20 + angle)); // probably not a +, idk
      bot.update();

      bot.followTrajectorySequenceAsync(traj2);

      while(bot.isBusy() && opModeIsActive()) { // !slides.atTarget && for feedforward, // slides.update(); in loop
         bot.update();
      }

   }

}


/*
public void deliverYellow() {
        Vector2d destination; //need to fix (offset)
        sleep(1000);
        fluffy.drive.pose = fluffy.getPoseFromAprilTag();
        if (PATH.equals("Left")) {
            destination = fluffy.tagPositions[0].plus(fluffy.DELIVERY_OFFSET_RED);
        } else if (PATH.equals("Center")) {
            destination = fluffy.tagPositions[1].plus(fluffy.DELIVERY_OFFSET_RED);
        } else {
            destination = fluffy.tagPositions[2].plus(fluffy.DELIVERY_OFFSET_RED);
        }
        if ((int) initMenu.get(1) == 1) { // allows our driver to select right or left delivery position
            destination = destination.plus(new Vector2d(0, 3.5));
        }
 */
