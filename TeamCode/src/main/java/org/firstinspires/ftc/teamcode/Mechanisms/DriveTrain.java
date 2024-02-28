package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.hardware.IMU;
public class DriveTrain {
    private final DcMotorEx motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;
    private final HardwareMap hardwareMap;
    private final Gamepad gamepad1;

    private Intake.IntakeStates intakeStates = Intake.IntakeStates.GROUND;

   // private static SampleMecanumDrive drive;

    private Telemetry telemetry;

    private double dPadControl = 0;//need to figure out actual values returned by controller through telemetry
    private IMU imu;
    private YawPitchRollAngles robotOrientation;
    private double Yaw;
    private double Pitch;
    private double Roll;
    private boolean tipping;

    private enum Mode{
        ROBOT_CENTRIC,
        FIELD_CENTRIC,
    }

    Mode currentMode = Mode.ROBOT_CENTRIC;
    public DriveTrain(HardwareMap hM, Telemetry telemetry, Gamepad gamepad) {

        this.hardwareMap = hM;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad;
        //drive = new SampleMecanumDrive(hardwareMap);

        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("FL");
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("BL");
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("FR");
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("BR");

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        imu = hardwareMap.get(IMU.class, "imu");

    }

    public void setIntake(Intake.IntakeStates i){
        intakeStates  = i;
    }
    //Need to do IMUSetup on Start!!
    public void IMUSetup(){
        IMU.Parameters myIMUParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(myIMUParameters);
        robotOrientation = imu.getRobotYawPitchRollAngles();
        Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
        Roll = robotOrientation.getRoll(AngleUnit.DEGREES);
        imu.resetYaw();
    }



    public void drive() {
        //dpad control for scoring use ig
        //antiTip();
        if(!tipping) {
            if (gamepad1.dpad_left) {
                dPadControl = -0.5;
            } else if (gamepad1.dpad_right) {
                dPadControl = 0.5;
            } else {
                dPadControl = 0;
            }
            /*
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x; */
            double y =  -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double frontLeftPower = 0, frontRightPower = 0, backLeftPower = 0, backRightPower = 0, denominator = 0 ;


            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            // Calculate the mecanum motor powers
            frontLeftPower = (y + x + 2 * rx) / denominator;
            backLeftPower = (y - x + 2 * rx) / denominator;
            frontRightPower = (y - x - 2 * rx) / denominator;
            backRightPower = (y + x - 2 * rx) / denominator;
            // Cube the motor powers
            frontLeftPower = Math.pow(frontLeftPower, 3);
            frontRightPower = Math.pow(frontRightPower, 3);
            backLeftPower = Math.pow(backLeftPower, 3);
            backRightPower = Math.pow(backRightPower, 3);
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]

            // Calculate the maximum value of all the motor powers
            // The argument here is just an array separated into different lines
            double maxValue = getMax(new double[]{
                    frontLeftPower,
                    frontRightPower,
                    backLeftPower,
                    backRightPower
                    /*Math.abs(frontLeftPower),
                    Math.abs(frontRightPower),
                    Math.abs(backLeftPower),
                    Math.abs(backRightPower)*/
            });

            // Resize the motor power values
            if (maxValue > 1) {
                frontLeftPower /= maxValue;
                frontRightPower /= maxValue;
                backLeftPower /= maxValue;
                backRightPower /= maxValue;
            }
            if(gamepad1.right_trigger>0.1){
                motorFrontLeft.setPower(frontLeftPower*.4);
                motorBackLeft.setPower(backLeftPower*.4);
                motorFrontRight.setPower(frontRightPower*.4);
                motorBackRight.setPower(backRightPower*.4);
            }
            /*
            else if(intakeStates==Intake.IntakeStates.INTAKE){//intakeStates==Intake.IntakeStates.INTAKE
                motorFrontLeft.setPower(frontLeftPower*.3);
                motorBackLeft.setPower(backLeftPower*.3);
                motorFrontRight.setPower(frontRightPower*.3);
                motorBackRight.setPower(backRightPower*.3);
            } */
            else{
                motorFrontLeft.setPower(frontLeftPower);
                motorBackLeft.setPower(backLeftPower);
                motorFrontRight.setPower(frontRightPower);
                motorBackRight.setPower(backRightPower);

            }
        }

    }





    private double getMax(double[] input) {
        double max = Integer.MIN_VALUE;
        for (double value : input) {
            if (Math.abs(value) > max) {
                max = Math.abs(value);
            }
        }
        return max;
    }

    public IMU getImu(){
        return imu;
    }
    public void antiTip() {
        Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
        if(Math.abs(Pitch)>10 && Math.abs(Pitch)<70){
            tipping = true;
            double pitchCorrection = -0.08*Pitch;  // tune 0.125 p term later
            motorBackLeft.setPower(pitchCorrection);
            motorBackRight.setPower(pitchCorrection);
            motorFrontLeft.setPower(pitchCorrection);
            motorFrontRight.setPower(pitchCorrection);
        }
        else {
            tipping = false;
        }
    }
    public boolean isTipping(){
        return tipping;
    }

}
