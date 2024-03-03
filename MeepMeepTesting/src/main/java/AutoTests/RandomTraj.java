package AutoTests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RandomTraj {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);
        Pose2d blueStart = new Pose2d(14, -61, Math.toRadians(90));

        RoadRunnerBotEntity spikeRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 40, 2.225, 1.895, 11.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(blueStart)
                                .lineToLinearHeading(new Pose2d(14, -35, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(55,-35,Math.toRadians(0)))
                                .build()

                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(spikeRight)
                .start();
    }
}
