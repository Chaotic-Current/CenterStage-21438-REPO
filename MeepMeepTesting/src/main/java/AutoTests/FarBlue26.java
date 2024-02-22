package AutoTests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class FarBlue26 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity spikeRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55, 55, Math.toRadians(110), Math.toRadians(110), 11.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(14, 61, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(14, 32, Math.toRadians(270)))
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(19, 40), Math.toRadians(0))
                                .lineToSplineHeading(new Pose2d(36, 40, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(45.5, 34), Math.toRadians(0))
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(40, 25, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(20,12), Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(0)))
                                .waitSeconds(1)
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(23, 12, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(42,27), Math.toRadians(45))
                                .waitSeconds(1)

                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(35, 20, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(20,12), Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(0)))
                                .waitSeconds(1)
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(23, 12, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(42,27), Math.toRadians(45))
                                .waitSeconds(1)



                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(spikeRight)
                .start();
    }
}
