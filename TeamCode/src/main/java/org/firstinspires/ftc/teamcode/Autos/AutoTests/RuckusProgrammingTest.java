package org.firstinspires.ftc.teamcode.Autos.AutoTests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Config
public class RuckusProgrammingTest extends LinearOpMode {
   Rev2mDistanceSensor left, right;
   private  double angle;
   private IMU imu;
   private ElapsedTime timer = new ElapsedTime();
   public static double offset = 3.8;

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
              .lineToLinearHeading(new Pose2d(10, 0, Math.toRadians(-20)))
              .waitSeconds(1.5)
              .build();

      bot.followTrajectorySequenceAsync(traj1);

      while(bot.isBusy() && opModeIsActive() && !isStopRequested()) { // !slides.atTarget && for feedforward, // slides.update(); in loop
         bot.update();
      }

      /* (ignore)
      timer.reset();
      while(timer.seconds() < 10){
         if(!bot.isBusy()){
            angle = Math.atan((right.getDistance(DistanceUnit.INCH) - left.getDistance(DistanceUnit.INCH)) / 11.2);
            TrajectorySequence traj2 = bot.trajectorySequenceBuilder(new Pose2d())
                    .turn(angle)
                    .build();
            bot.followTrajectorySequenceAsync(traj2);
         }
         bot.update();
      } */

      angle = Math.atan((right.getDistance(DistanceUnit.INCH) - left.getDistance(DistanceUnit.INCH)) / 11.2)+Math.toRadians(offset);
      imu.resetYaw();
      TrajectorySequence traj2 = bot.trajectorySequenceBuilder(new Pose2d())
              //.lineToLinearHeading(new Pose2d(15, 20, bot.getPoseEstimate().getHeading() - angle))
              .turn(angle)
              .waitSeconds(1.0)
              .lineToLinearHeading(new Pose2d(-5, 15, Math.toRadians(90)))
              .build();

       bot.followTrajectorySequenceAsync(traj2);

      while(bot.isBusy() && opModeIsActive()) { // !slides.atTarget && for feedforward, // slides.update(); in loop
         bot.update();
         telemetry.addLine("angle (degrees): " + Math.toDegrees(angle));
         telemetry.update();
      }

   }

}
