package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mechanisms.KalmanF;
import org.firstinspires.ftc.teamcode.Mechanisms.MechanismTests.AprilTagCam;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->

 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */

@Config
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    private double errorX, errorY;
    public static double TICKS_PER_REV = 8192; //2000
    public static double WHEEL_RADIUS = 0.688976378; // in 0.688976378 .944882
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = -1.85; // -2.5 X is the up and down direction
    public static double PARALLEL_Y = 4.61; // -4.85 Y is the strafe direction

    public static double PERPENDICULAR_X = -1.75; //7
    public static double PERPENDICULAR_Y = -1.4;//-3
    public static double X_MULTIPLIER = 0.99338783679;// 0.99338783679   ((50.0/50.3882)+(50.0/50.4448)+(50.0/50.5929))/3.0
    public static double Y_MULTIPLIER = 1.0063490194;//1.0063490194     ((50.0/50.0286)+(50.0/50.6799)+(50.0/50.7853))/3.0
    public static double xOffset = 0;
    public static double yOffset = 0;
    public  static boolean readyToScan = false;

    public static boolean hasScanned = false;


    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private Encoder parallelEncoder, perpendicularEncoder;

    private SampleMecanumDrive drive;

    private KalmanF multiSensorFuser;

    private SimpleMatrix measurements;

    private AprilTagCam frontTagCam;

    private enum CamStates{
        FRONTON,
        OFF,
        BACKON
    }

    private CamStates currentCamState = CamStates.FRONTON;
    private ArrayList<Double> frontBoardInfo;


    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.drive = drive;

       // multiSensorFuser = new KalmanF(new Pose2d(0,0,0));
     //   frontTagCam = new AprilTagCam(hardwareMap,"WebcamFront",2);
        //HAVE TO INTIALIZE telemetry -> bot.getEncoder.setCamTelemetry(telemetry);

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "PO"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "IN"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        errorX = 0;
        errorY = 0;
    }

    public double getAverage(double[] x) {
        double sum = 0;
        for (double t : x) {
            sum += t;
        }

        return sum / x.length;
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        //Saving current positions of encoders
        /*
        double perpendicularPos = encoderTicksToInches(perpendicularEncoder.getCurrentPosition()) * X_MULTIPLIER;
        double parallelPos = encoderTicksToInches(parallelEncoder.getCurrentPosition()) * Y_MULTIPLIER;

        //updating apriltag data
        frontTagCam.aprilTagUpdate();
        //applying apriltag to kalman filter through measurements state vector
        switch(currentCamState){
            case OFF:
                measurements = new SimpleMatrix(new double[][]{
                        {perpendicularPos},
                        {parallelPos}, // parallelPos+frontTagCam.getDepthDisplacement(frontBoardInfo) boardInfo can be altered using setBoardInfo()
                        {getWheelVelocities().get(1)},
                        {getWheelVelocities().get(0)}
                });
                break;
            case FRONTON:
                measurements = new SimpleMatrix(new double[][]{
                        {frontTagCam.getHorizDisplacement()},
                        {parallelPos}, // parallelPos+frontTagCam.getDepthDisplacement(frontBoardInfo) boardInfo can be altered using setBoardInfo()
                        {0},
                        {0}
                });
                break;
            case BACKON:
                measurements = new SimpleMatrix(new double[][]{
                        {0},
                        {1}, //boardInfo can be altered using setBoardInfo()->preferebly only altered once in init
                        {0},
                        {0}
                });
                break;

        }
        //Kalman Filter Input update method
        Pose2d inputs = new Pose2d(getWheelVelocities().get(1),getWheelVelocities().get(0),0);
        multiSensorFuser.inputUpdate(inputs, measurements);
            */


        return Arrays.asList(
                (encoderTicksToInches(parallelEncoder.getCurrentPosition())) * X_MULTIPLIER ,
                (encoderTicksToInches(perpendicularEncoder.getCurrentPosition())) * Y_MULTIPLIER
        );

    }

    public double getParallelEncoderPos(){
        return (encoderTicksToInches(parallelEncoder.getCurrentPosition())) * X_MULTIPLIER;
    }

    public double getPerpendicularEncoderPos(){
        return (encoderTicksToInches(perpendicularEncoder.getCurrentPosition())) * Y_MULTIPLIER;
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }

    public double getWheelVelocitySum(){return encoderTicksToInches(parallelEncoder.getCorrectedVelocity()) * X_MULTIPLIER + encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity()) * Y_MULTIPLIER;}

    public void setErrorX(double errorX) {
        this.errorX = errorX;
    }

    public double getErrorX() {
        return errorX;
    }

    public void setErrorY(double errorY) {
        this.errorY = errorY;
    }

    public double getErrorY() {
        return errorY;
    }

    public void setCamTelemetry(Telemetry tele){
        frontTagCam.setTelemetry(tele);
    }

    public void setFrontBoardInfo(ArrayList<Double> list){
        // This is basically (boardPosition(1)-currentPose(0))-(camDepth(2));
        //TeleOp init: bot.getEncoder.setFrontBoardInfo(new ArrayList<>(Arrays.asList(currentPos,boardPos,camDepth)));
        frontBoardInfo = new ArrayList<Double>(Arrays.asList(
                list.get(0),
                list.get(1),
                list.get(2)
        ));
    }
    public KalmanF getMultiSensorFuser(){
        return multiSensorFuser;
    }

    public void activateFrontCam(){
        currentCamState = CamStates.FRONTON;

    }
}