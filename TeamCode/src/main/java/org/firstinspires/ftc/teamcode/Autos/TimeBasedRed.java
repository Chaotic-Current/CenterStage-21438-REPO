package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Pipelines.DetectRed;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous (name = "fucked")
@Config
public class TimeBasedRed extends LinearOpMode {

    private OpenCvWebcam backCam;
    private int width;
    public static double powerLeft = 0.25, powerRight = 0.25, powerForward = 0.25;
    public static int timeLeft1 = 10000,timeLeft2 = 10000, timeForward = 777, timeRight1= 100000, timeRight2 = 100000, timeRight3 = 100000;//12.6
    private DetectRed detector;
    private int height;
    private DcMotorEx motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;
    private Servo left, right;

    public void Initialize(){
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("FL"); // Pin 0
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("BL"); // Pin 1
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("FR"); // Pin 2
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("BR"); // Pin 3

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Reverse motors
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        left = hardwareMap.get(Servo.class, "ARM_L");
        right = hardwareMap.get(Servo.class, "ARM_R");

        left.setPosition(0.8); // left start pos
        right.setPosition(0.2); // right start pos

        width = 160;
        height = 120;
        detector = new DetectRed(width, telemetry);

        // Initialize the back-facing camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        backCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        backCam.setPipeline(detector);

        backCam.setMillisecondsPermissionTimeout(2500);

    }
    @Override
    public void runOpMode() throws InterruptedException {
        Initialize();

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


        waitForStart();

        DetectRed.RedLocation e = detector.getLocate();

        backCam.stopStreaming();

        switch (e){
            case LEFT:
                caseLeft();
            case RIGHT:
                caseRight();
            case CENTER:
                caseCenter();
            case UNDETECTED:
                caseCenter();
            default:
                telemetry.addLine("Bruh");
        }

    }

    public void caseLeft(){
        ElapsedTime timer = new ElapsedTime();

        while(timer.milliseconds() <= timeLeft1){
            motorFrontLeft.setPower(-powerLeft);
            motorBackRight.setPower(-powerLeft);
            motorBackLeft.setPower(powerLeft);
            motorFrontRight.setPower(powerLeft);
        }

        timer.reset();

        while (timer.milliseconds() <= timeLeft2){
            motorFrontLeft.setPower(powerLeft);
            motorBackRight.setPower(powerLeft);
            motorBackLeft.setPower(powerLeft);
            motorFrontRight.setPower(powerLeft);
        }
    }

    public void caseRight(){
        ElapsedTime timer = new ElapsedTime();

        while (timer.milliseconds() <= timeRight1){
            motorFrontLeft.setPower(powerRight);
            motorBackRight.setPower(powerRight);
            motorBackLeft.setPower(powerRight);
            motorFrontRight.setPower(powerRight);
        }

        timer.reset();

        while(timer.milliseconds() <= timeRight2){
            motorFrontLeft.setPower(-powerRight);
            motorBackRight.setPower(powerRight);
            motorBackLeft.setPower(-powerRight);
            motorFrontRight.setPower(powerRight);
        }

        timer.reset();

        while (timer.milliseconds() <= timeRight3){
            motorFrontLeft.setPower(powerRight);
            motorBackRight.setPower(powerRight);
            motorBackLeft.setPower(powerRight);
            motorFrontRight.setPower(powerRight);
        }
    }

    public void caseCenter(){
        ElapsedTime timer = new ElapsedTime();

        while (timer.milliseconds() <= timeForward){
            motorFrontLeft.setPower(powerForward);
            motorBackRight.setPower(powerForward);
            motorBackLeft.setPower(powerForward);
            motorFrontRight.setPower(powerForward);
        }
    }
}
