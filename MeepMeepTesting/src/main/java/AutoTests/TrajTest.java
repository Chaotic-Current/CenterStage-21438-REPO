package AutoTests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class TrajTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);
        Pose2d blueStart = new Pose2d(19.6, 61, Math.toRadians(270));

        RoadRunnerBotEntity spikeRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 40, 2.225, 1.895, 11.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(blueStart)
                                .lineToLinearHeading(new Pose2d(14, 44, Math.toRadians(270)))
                                .splineToSplineHeading(new Pose2d(5, 33, Math.toRadians(210)), Math.toRadians(225))
                                .setReversed(true)
                                .back(10) //tune this distance frfr
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //extend outtake
                                })
                                .lineToConstantHeading(new Vector2d(19, 40))
                                .lineToSplineHeading(new Pose2d(36, 40, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(60, 31), Math.toRadians(0))
                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                                    //open claw
                                })
                                .waitSeconds(2)
                                //relocalize
                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                                    //retract outtake
                                })
                                .lineToLinearHeading(new Pose2d(40, 25, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(20,12), Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(0)))
                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                                    //run intake
                                })
                                .waitSeconds(2)
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(23, 12, Math.toRadians(0)))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //extend outtake
                                })
                                .splineToConstantHeading(new Vector2d(42,31), Math.toRadians(45))
                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                                    //open claw
                                })
                                .waitSeconds(2)
                                //relocalize
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //retract outtake
                                    //Pose2d traj2StartPosewOffset = new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY()- frontTagCam.getHorizDisplacement(),drive.getPoseEstimate().getHeading());
                                    // drive.setPoseEstimate(traj2StartPosewOffset);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                                    //retract outtake
                                })
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(35, 20, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(20,12), Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(0)))
                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                                    //run intake
                                })
                                .waitSeconds(2)
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(23, 12, Math.toRadians(0)))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //extend outtake
                                })
                                .splineToConstantHeading(new Vector2d(42,27), Math.toRadians(45))
                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                                    //open claw
                                })
                                .waitSeconds(2)
                                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                                    //retract outtake
                                })

                                .back(7)
                                .build()

                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(spikeRight)
                .start();
    }
}
