package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MechanismTemplates.ArmPID;
import org.firstinspires.ftc.teamcode.MechanismTemplates.ClawMech;
import org.firstinspires.ftc.teamcode.MechanismTemplates.SlideMech;
import org.firstinspires.ftc.teamcode.Pipelines.DetectColor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
@Config
public class RedCloseSide extends LinearOpMode {

    public static int location = 1;
    private SampleMecanumDrive drive;
    private OpenCvWebcam frontCam, backCam;
    private ArmPID arm;
    private SlideMech slide;
    private ClawMech clawMech;
    private DetectColor detector; //This will be out of the frame for know, along with the april tag pipeline
    //TODO all these values are just place holders so the code doesn't cause an error, delete this comment after you see it
    public static double frwDistance1 = 24;
    public static double linetoLinear1X = 24, linetoLinear1Y = -24, lineToLinear1Heading = -30;
    public static double splineToLinear1X = 24, splineToLinear1Y = 24, splineToLinear1Heading = 80;
    public static double splineToLinear2X = 24, splineToLinear2Y = -32, splineToLinear2Heading = -90, wait1 = 3;
    public static double splineToLinear3X = 16, splineToLinear3Y = -32, splineToLinear3Heading = -90, wait2 = 3;
    public static double splineToLinear4X = 8, splineToLinear4Y = -32, splineToLinear4Heading = -90, wait3 = 3;

    //For now, ignore this, it is not gonna be in use for now, just manually change spike locations for now
  /*  public void cameraInit(){
        int width = 160;
        int height = 120;

        detector = new DetectColor(width, telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        backCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "WebcamBack"), cameraMonitorViewId);
        frontCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "WebcamFront"), cameraMonitorViewId);
        backCam.setPipeline(detector);
        frontCam.setPipeline(detector);

        backCam.setMillisecondsPermissionTimeout(2500);
        frontCam.setMillisecondsPermissionTimeout(2500);
        backCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                telemetry.addLine("started");
                backCam.startStreaming(160, 120, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("not open");
            }
        });
        frontCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                telemetry.addLine("started");
                backCam.startStreaming(160, 120, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("not open");
            }
        });
    }*/

    public void initialize(){
        drive = new SampleMecanumDrive(hardwareMap);
        arm = new ArmPID(hardwareMap);
        slide = new SlideMech(hardwareMap);
        clawMech = new ClawMech(hardwareMap);
        //DO NOT UNCOMMENT THIS
        //cameraInit();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        //this will be camera vision based later
        //For now, 1 is left, 2 is center, 3 is right

        TrajectorySequence firstMove;
        TrajectorySequence moveToBackboard;

        //I added what I think would be proper points for mechanisms to do what they need to do, but I commented them out just for now
        switch (location){
            case 1:
                firstMove = drive.trajectorySequenceBuilder(new Pose2d())
                        .splineToLinearHeading(new Pose2d(splineToLinear1X,splineToLinear1Y,Math.toRadians(splineToLinear1Heading)), Math.toRadians(0))
                        .build();

                moveToBackboard = drive.trajectorySequenceBuilder(firstMove.end())
                        .splineToLinearHeading(new Pose2d(splineToLinear2X,splineToLinear2Y,Math.toRadians(splineToLinear2Heading)), Math.toRadians(0))
                        .UNSTABLE_addTemporalMarkerOffset(wait1, () ->{
                            //slide.setLowJunction();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(wait1+1,() ->{
                            //arm.setExtakeOrIntake();
                        })
                        .waitSeconds(4)
                        .UNSTABLE_addTemporalMarkerOffset(wait1+2,() ->{
                            //clawMech.setBothOpen(false);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(wait1+2.5,() ->{
                            //arm.setExtakeOrIntake();
                            //clawMech.getClaw().setPosition(clawMech.close);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(wait1+3.5,() ->{
                            //slide.setIntakeOrGround();
                        })
                        .build();
            case 2:
                firstMove = drive.trajectorySequenceBuilder(new Pose2d())
                        .forward(frwDistance1)
                        .build();
                moveToBackboard = drive.trajectorySequenceBuilder(firstMove.end())
                        .splineToLinearHeading(new Pose2d(splineToLinear3X,splineToLinear3Y,Math.toRadians(splineToLinear3Heading)), Math.toRadians(0))
                        .UNSTABLE_addTemporalMarkerOffset(wait2, () ->{
                            //slide.setLowJunction();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(wait2+1,() ->{
                            //arm.setExtakeOrIntake();
                        })
                        .waitSeconds(4)
                        .UNSTABLE_addTemporalMarkerOffset(wait2+2,() ->{
                            //clawMech.setBothOpen(false);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(wait2+2.5,() ->{
                            //arm.setExtakeOrIntake();
                            //clawMech.getClaw().setPosition(clawMech.close);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(wait2+3.5,() ->{
                            //slide.setIntakeOrGround();
                        })
                        .build();
            case 3:
                firstMove = drive.trajectorySequenceBuilder(new Pose2d())
                        .lineToLinearHeading(new Pose2d(linetoLinear1X,linetoLinear1Y,Math.toRadians(lineToLinear1Heading)))
                        .build();

                moveToBackboard = drive.trajectorySequenceBuilder(firstMove.end())
                        .splineToLinearHeading(new Pose2d(splineToLinear4X,splineToLinear4Y,Math.toRadians(splineToLinear4Heading)), Math.toRadians(0))
                        .UNSTABLE_addTemporalMarkerOffset(wait3, () ->{
                            //slide.setLowJunction();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(wait3+1,() ->{
                            //arm.setExtakeOrIntake();
                        })
                        .waitSeconds(4)
                        .UNSTABLE_addTemporalMarkerOffset(wait3+2,() ->{
                            //clawMech.setBothOpen(false);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(wait3+2.5,() ->{
                            //arm.setExtakeOrIntake();
                            //clawMech.getClaw().setPosition(clawMech.close);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(wait3+3.5,() ->{
                            //slide.setIntakeOrGround();
                        })
                        .build();
            default:
                firstMove = drive.trajectorySequenceBuilder(new Pose2d())
                        .forward(frwDistance1)
                        .build();
                moveToBackboard = drive.trajectorySequenceBuilder(firstMove.end())
                        .splineToLinearHeading(new Pose2d(splineToLinear3X,splineToLinear3Y,Math.toRadians(splineToLinear3Heading)), Math.toRadians(0))
                        .build();
        }


        waitForStart();

        drive.followTrajectorySequenceAsync(firstMove);
        drive.followTrajectorySequenceAsync(moveToBackboard);

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            slide.update(telemetry);
            arm.update(telemetry, new ElapsedTime());

        }
    }

}
