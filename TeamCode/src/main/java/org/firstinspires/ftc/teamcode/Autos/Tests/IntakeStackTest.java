package org.firstinspires.ftc.teamcode.Autos.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.ClawMech;
import org.firstinspires.ftc.teamcode.Mechanisms.IntakeMech;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
@Disabled
public class IntakeStackTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        IntakeMech intake = new IntakeMech(hardwareMap,telemetry);
        intake.setServosUp();
        ClawMech clawMech = new ClawMech(hardwareMap,telemetry);
        clawMech.open();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d())
                .UNSTABLE_addDisplacementMarkerOffset(0,()->{
                    intake.startAuto();
                })
                .UNSTABLE_addTemporalMarkerOffset(1,()->{
                   // intake.setLowering();
                })
                .UNSTABLE_addTemporalMarkerOffset(2,()->{
                    //intake.stop();
                })
                .waitSeconds(10)
                .back(0.1)
                .build();

        waitForStart();
        drive.followTrajectorySequenceAsync(traj);

        while (opModeIsActive() && !isStopRequested()){
            intake.update(drive);
            drive.update();
            telemetry.update();
        }
    }
}
