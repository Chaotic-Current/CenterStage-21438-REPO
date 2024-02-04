package org.firstinspires.ftc.teamcode.Mechanisms.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Pipelines.DetectJunction;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


public class CameraTest extends OpMode {

    private int width;
    private int height;

    private double focalLenght;
    private DetectJunction detector;
    private OpenCvWebcam backCam;
    @Override
    public void init() {
        width = 160;
        height = 120;
        focalLenght = 1430;
        detector = new DetectJunction(telemetry,focalLenght);

        // Initialize the back-facing camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        backCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "WebcamFront"), cameraMonitorViewId);
        backCam.setPipeline(detector);

        backCam.setMillisecondsPermissionTimeout(2500);
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

        telemetry.addLine("Waiting for start");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addLine("Angle: " + detector.getAngle());
    }
}
