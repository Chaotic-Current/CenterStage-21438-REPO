package org.firstinspires.ftc.teamcode.Autos.Stack_Autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.Camera;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.MechanismTests.AprilTagCam;
import org.firstinspires.ftc.teamcode.Mechanisms.Outake;
import org.firstinspires.ftc.teamcode.Pipelines.DetectColor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "BlueFarStack(NoReloc)")
@Config

public class NoRelocBlueFarStack extends LinearOpMode {

    enum States {
        START,

        TO_BACKDROP_GOING_TO_STACK,

        TO_STACK_GOING_TO_BACKDROP,
        IDLE
    }

    States currentState = States.START;
    SampleMecanumDrive drive;

    Camera.PropDetection PropDetection;

    private AprilTagCam frontTagCam;
    private OpenCvWebcam cam;

    DetectColor detector;
    Outake outake;
    Intake intake;
    public static double boardX = 44;
    public static double iterations = 2;

    public static double tagID = 3;
    public static double tagLocation = 30;

    public static double camMultiplier = 0.75;



    public void cameraInit() {
        frontTagCam = new AprilTagCam(hardwareMap,"WebcamFront",tagID);
        frontTagCam.setTelemetry(telemetry);
    }


    public void initialize() {
        drive = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap,telemetry,outake);
        cameraInit();
    }


    @Override
    public void runOpMode() {
        initialize();
        Pose2d blueStart = new Pose2d(18.1, 59, Math.toRadians(270)); // x:14 y:61 heading: 270

        drive.setPoseEstimate(blueStart);

        TrajectorySequence firstBlueTraj = drive.trajectorySequenceBuilder(blueStart)
                .lineToLinearHeading(new Pose2d(14, 45, Math.toRadians(270)))
                .splineToSplineHeading(new Pose2d(17, 38, Math.toRadians(315)), Math.toRadians(315))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(14, 45, Math.toRadians(315)))
                .lineToSplineHeading(new Pose2d(33, 47, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(50, 38), Math.toRadians(0))
                .waitSeconds(1)

                //2+0

                .setReversed(true)
                //.splineToConstantHeading(new Vector2d(20,12), Math.toRadians(180))
                .lineToSplineHeading(new Pose2d(30,11,Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(-55.0, 11, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    //run intake
                    intake.setToIntake(0.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    intake.setToIntake(0.79);
                })
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    intake.setToIntake(0.77);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{
                    intake.setToIntake(0.8);
                })
                .waitSeconds(1)

                .setReversed(false)
                .lineToLinearHeading(new Pose2d(30, 11, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    intake.setToGround();
                })
                .lineToLinearHeading(new Pose2d(42,30,Math.toRadians(0)))
                //.splineToConstantHeading(new Vector2d(42,30), Math.toRadians(45))
                .waitSeconds(1)

                //2+2

                .setReversed(true)
                //.splineToConstantHeading(new Vector2d(20,12), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(30,13,Math.toRadians(0))) //12
                .lineToSplineHeading(new Pose2d(-55.5, 13, Math.toRadians(0))) //12
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    //run intake
                    intake.setToIntake(0.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    intake.setToIntake(0.76);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()->{
                    intake.setToIntake(0.74);
                })
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    intake.setToIntake(0.8);
                })
                .waitSeconds(1)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(30, 12, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    intake.setToGround();
                })
                .lineToLinearHeading(new Pose2d(43,30,Math.toRadians(0)))
                .waitSeconds(1)

                //2+4



                .build();

        TrajectorySequence toStack = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(0)))
                .build();;

        TrajectorySequence toBackdrop = drive.trajectorySequenceBuilder(firstBlueTraj.end())
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(23, 12, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(42,27), Math.toRadians(45))
                .waitSeconds(1)
                .build();



        currentState = States.START;

        while (opModeInInit()) {

        }

        int currentIteration = 0;
        while (opModeIsActive()) {
            intake.executeAuto();
            switch (currentState) {
                case START:
                    //Starts following the first trajectory and changes to backdrop state
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(firstBlueTraj);
                        currentState = States.IDLE;
                        currentIteration++;
                    }
                    break;
                case TO_BACKDROP_GOING_TO_STACK:
                    /*
                    From StartState/ReturnState: After following the first trajectory is done->(when at return, we need to go to backdrop),
                                     (1).Uses April tag data to make a new fixed pose estimate of the robot
                                     (2).Uses the new fixed Pose in drive.setPoseEstimate and in making a new traj so the bot's currentPosition and start pose for traj is same.
                                     (3).Starts Following path to backdrop
                                     (4).Adds to current iteration
                                     (5).Changes State to Return


                    */

                    if (!drive.isBusy()) {
                        //calculating the offset( apriltag offset - (our calculated odometry displacement from the tag)) to get how much we are off from our path
                       // double offset = calculateOffset(tagLocation,drive.getPoseEstimate().getY(),frontTagCam.getHorizDisplacement());
                        double yPos = frontTagCam.getHorizDisplacement() == 0 ? 25 : frontTagCam.getHorizDisplacement(); //toBackdrop.end().getY()
                        Pose2d fixedPose =  new Pose2d(toBackdrop.end().getX(), yPos, toBackdrop.end().getHeading());
                      //  drive.setPoseEstimate(fixedPose);
                        //try to keep sequences relatively short, like 5-6 commands at most, so build time isnt too long
                        toStack =  drive.trajectorySequenceBuilder(fixedPose)
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(40, 25, Math.toRadians(0)))
                                //.splineToConstantHeading(new Vector2d(20,12), Math.toRadians(180))
                                .lineToSplineHeading(new Pose2d(20,12,Math.toRadians(0))) //12
                                .lineToSplineHeading(new Pose2d(-60, 12, Math.toRadians(0))) //12
                                .waitSeconds(1)
                                .build();
                        drive.followTrajectorySequenceAsync(toStack);
                        currentIteration++;
                        currentState = States.TO_STACK_GOING_TO_BACKDROP;

                    }
                    break;
                case TO_STACK_GOING_TO_BACKDROP:
                     /*
                    From BackDropState: After following the second Trajectory(towards backdrop) is done->(When At Backdrop, we need to return)
                                     (1).Follows the premade third trajectory(trajectory that goes back couple of units)
                                     (2).If the number of iterations(cycles) the bot has done is greater than the num of iterations we specified
                                         then current state is changed to Idle(stop) state, else current state is changed back to backdrop



                    */
                    if (!drive.isBusy()) {
                        toBackdrop = drive.trajectorySequenceBuilder(firstBlueTraj.end())
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(23, 12, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(42,30), Math.toRadians(45))
                                .waitSeconds(1)
                                .build();
                        drive.followTrajectorySequenceAsync(toBackdrop);
                        if(currentIteration>iterations)
                            currentState = States.IDLE;
                        else
                            currentState = States.TO_BACKDROP_GOING_TO_STACK;
                    }
                    break;
                case IDLE:

                    break;

            }
            drive.update();
            frontTagCam.aprilTagUpdate();
            telemetry.update();

        }
    }
    public static double calculateOffset(double tagPosition, double currentPosition, double camData){
        return tagPosition-camData; //camMultiplier*(camData-(currentPosition-tagPosition))
    }
}