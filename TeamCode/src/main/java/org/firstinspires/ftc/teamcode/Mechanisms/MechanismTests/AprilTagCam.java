package org.firstinspires.ftc.teamcode.Mechanisms.MechanismTests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Mechanisms.LowPass;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@Config

public class AprilTagCam {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private LowPass lowP;

    public static double alp = 0.5;

    private double horizDisplacement = 0;
    private double depthDisplacement = 0;

    private double targetID = 0;
    public AprilTagCam(HardwareMap hw,String name,double targetID){
        this.hardwareMap = hw;

        aprilTag = new AprilTagProcessor.Builder().build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, name)); //"WebcamFront"

        builder.addProcessor(aprilTag);

        this.targetID = targetID;
        lowP = new LowPass(0,alp);

        visionPortal = builder.build();
    }
    public void aprilTagUpdate(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        //setting displacement values to 0, so when not picking up anything, the values added to Kalman fitler are 0


        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == targetID) {
                horizDisplacement = lowP.execute(detection.ftcPose.x);
                depthDisplacement = detection.ftcPose.y - 10;
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y-10, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry

    }

    public void setTelemetry(Telemetry tele){
        this.telemetry  = tele;
    }
    public void setTargetID(double id){
        this.targetID  = id;
    }

    public double getHorizDisplacement(){
        return horizDisplacement;
    }
    public double getDepthDisplacement(ArrayList<Double> boardDisplacmentInfo){
        // This is basically (boardPosition-currentPose)-(camDepth);
        return (boardDisplacmentInfo.get(1)- boardDisplacmentInfo.get(0))-(boardDisplacmentInfo.get(2));

    }
}
