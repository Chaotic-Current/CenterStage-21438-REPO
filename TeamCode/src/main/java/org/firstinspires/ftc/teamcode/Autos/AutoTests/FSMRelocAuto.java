package org.firstinspires.ftc.teamcode.Autos.AutoTests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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


@Autonomous(name = "FSMRelocAuto")
@Config

public class FSMRelocAuto extends LinearOpMode {

    enum States {
        START,

        GOING_TO_BACKDROP,

        GOING_TO_RETURN,
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

    public static double tagID = 2;
    public static double tagLocation = -35;

    public static double camMultiplier = 0.75;

    public void cameraInit() {
        frontTagCam = new AprilTagCam(hardwareMap,"WebcamFront",tagID);
        frontTagCam.setTelemetry(telemetry);
    }


    public void initialize() {
        drive = new SampleMecanumDrive(hardwareMap);
        cameraInit();
    }


    @Override
    public void runOpMode() {
        initialize();
        Pose2d blueStart = new Pose2d(14, -60.5, Math.toRadians(90));

        drive.setPoseEstimate(blueStart);

        TrajectorySequence firstBlueTraj = drive.trajectorySequenceBuilder(blueStart)
                .lineToLinearHeading(new Pose2d(14, -35, Math.toRadians(0)))
                .waitSeconds(4)
                .build();

        TrajectorySequence secondBlueTraj = drive.trajectorySequenceBuilder(firstBlueTraj.end())
                .lineToLinearHeading(new Pose2d(45,-35,Math.toRadians(0)))
                .build();

        TrajectorySequence thirdBlueTraj = drive.trajectorySequenceBuilder(secondBlueTraj.end())
                .lineToLinearHeading(new Pose2d(14,-35,Math.toRadians(0)))
                .build();

        currentState = States.START;

        while (opModeInInit()) {

        }

        int currentIteration = 0;
        while (opModeIsActive()) {
            switch (currentState) {
                case START:
                    //Starts following the first trajectory and changes to backdrop state
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(firstBlueTraj);
                        currentState = States.GOING_TO_BACKDROP;
                        currentIteration++;
                    }
                    break;
                case GOING_TO_BACKDROP:
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
                        double offset = calculateOffset(tagLocation,drive.getPoseEstimate().getY(),frontTagCam.getHorizDisplacement());
                        Pose2d fixedPose =  new Pose2d(thirdBlueTraj.end().getX(),thirdBlueTraj.end().getY()+offset,thirdBlueTraj.end().getHeading());
                        drive.setPoseEstimate(fixedPose);
                        //try to keep sequences relatively short, like 5-6 commands at most, so build time isnt too long
                        secondBlueTraj =  drive.trajectorySequenceBuilder(fixedPose)
                                .lineToLinearHeading(new Pose2d(55,-35,Math.toRadians(0)))
                                .build();
                        drive.followTrajectorySequenceAsync(secondBlueTraj);
                        currentIteration++;
                        currentState = States.GOING_TO_RETURN;

                    }
                    break;
                case GOING_TO_RETURN:
                     /*
                    From BackDropState: After following the second Trajectory(towards backdrop) is done->(When At Backdrop, we need to return)
                                     (1).Follows the premade third trajectory(trajectory that goes back couple of units)
                                     (2).If the number of iterations(cycles) the bot has done is greater than the num of iterations we specified
                                         then current state is changed to Idle(stop) state, else current state is changed back to backdrop



                    */
                    if (!drive.isBusy()) {
                        /*thirdBlueTraj = drive.trajectorySequenceBuilder(secondBlueTraj.end())
                                .lineToLinearHeading(new Pose2d(14,-35,Math.toRadians(0)))
                                .build(); */
                        drive.followTrajectorySequenceAsync(thirdBlueTraj);
                        if(currentIteration>iterations)
                            currentState = States.IDLE;
                        else
                            currentState = States.GOING_TO_BACKDROP;
                    }
                    break;
                case IDLE:

                    break;
                    
            }
            drive.update();
            frontTagCam.aprilTagUpdate();

        }
    }
    public static double calculateOffset(double tagPosition, double currentPosition, double camData){
        return camMultiplier*(camData-(currentPosition-tagPosition));
    }
}