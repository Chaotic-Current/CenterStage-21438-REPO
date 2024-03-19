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
                .setConstraints(55, 55, Math.toRadians(85), Math.toRadians(50), 11.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-40, 61, Math.toRadians(270)))


                                .splineToSplineHeading(new Pose2d(-30, 34, Math.toRadians(315)), Math.toRadians(315))
                                .back(4)
                                .lineToSplineHeading(new Pose2d(-54.5, 35, Math.toRadians(0)))



                                /*

                                .lineToLinearHeading(new Pose2d(-36,32,Math.toRadians(270)))
                                .back(4)
                                .setReversed(true)
                                .lineToSplineHeading(new Pose2d(-54.5,35,Math.toRadians(0)))

                                */

                                /*
                                .lineToLinearHeading(new Pose2d(-46,33,Math.toRadians(270)))
                                .back(4)
                                .lineToSplineHeading(new Pose2d(-54.5,35,Math.toRadians(0)))
                                .waitSeconds(1)

                                */

                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(-35,58.5),Math.toRadians(0))
                                .lineToSplineHeading(new Pose2d(25,58.5,Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(45,40),Math.toRadians(0))

                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(30,58.5),Math.toRadians(180))
                                .back(2)
                                .lineToSplineHeading(new Pose2d(-20,58.5,Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(-54,35),Math.toRadians(180))
                                .lineToSplineHeading(new Pose2d(-56.5,35,Math.toRadians(0)))

                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(spikeRight)
                .start();
    }
}
