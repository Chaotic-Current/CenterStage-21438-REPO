package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Pipelines.DetectAprilTag;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class AprilTagTest extends LinearOpMode {

    private Pose2d x = new Pose2d();
    private SampleMecanumDrive drive;

    private double correctoin;

    DetectAprilTag detectAprilTag;

    public void initialize(){
        drive = new SampleMecanumDrive(hardwareMap);
        detectAprilTag = new DetectAprilTag();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();


        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        TrajectorySequence moveForward = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(2)
                .build();
        correctoin = detectAprilTag.getY();

        x = moveForward.end();

        TrajectorySequence retest = drive.trajectorySequenceBuilder(x)
                .lineTo(new Vector2d(12,0))
                .build();

        waitForStart();

        drive.followTrajectorySequence(moveForward);

        x = new Pose2d(x.getX(),x.getY()+detectAprilTag.getY(),0);

        drive.followTrajectorySequence(retest);


    }
}
