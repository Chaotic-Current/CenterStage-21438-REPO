package org.firstinspires.ftc.teamcode.Autos.AutoTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
@Autonomous(name="HuskyLensTest")
@Disabled
public class HuskyLensTest extends LinearOpMode {

    OpenCvCamera camera;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "huskylens"), cameraMonitorViewId);
        camera.openCameraDevice();
        camera.setPipeline(new HuskyLensPipeline());
        camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while (opModeIsActive() && (getRuntime() < 30)) {
            // fghjkl
        }
        camera.closeCameraDevice();

    }


    class HuskyLensPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {
            // add code here?
            return input;
        }
    }
}