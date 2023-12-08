package AutoTests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueCloseTests {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity spikeRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 40, 2.225, 1.895, 11.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 62, Math.toRadians(270)))
                                .forward(15) //+15 x
                                .splineTo(new Vector2d(7,33),Math.toRadians(220)) //+5 y, +14 x, +50 degree angle
                                .lineToLinearHeading(new Pose2d(12,40,Math.toRadians(270)))//-5 y, -7 x, - 50 degree angle
                                .splineTo(new Vector2d(49,28.5),Math.toRadians(360))//-37 y, + 11.5 x, - 90 degree angle
                                .waitSeconds(2)
                                .lineToLinearHeading(new Pose2d(39,12, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-59,11.5, Math.toRadians(180)))
                                .waitSeconds(5)
                                .lineToLinearHeading(new Pose2d(39,12, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(new Vector2d(49,28.5),Math.toRadians(360)))
                                .build()
                );

        RoadRunnerBotEntity spikeleft = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 40, 2.225, 1.895, 11.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 62, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(23,39,Math.toRadians(270)))
                                .back(13)
                                .lineToLinearHeading(new Pose2d(49,42,Math.toRadians(360)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(spikeRight)
                .start();
    }
}
