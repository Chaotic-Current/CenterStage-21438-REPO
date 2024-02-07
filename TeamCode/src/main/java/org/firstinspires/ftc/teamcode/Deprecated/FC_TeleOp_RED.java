package org.firstinspires.ftc.teamcode.Deprecated;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class FC_TeleOp_RED extends LinearOpMode {
    public final double TURN_PRECISION = 0.65;

    private final double PRECISIONREDUCTION = 0.39;

    private double getMax(double[] input) {
        double max = Integer.MIN_VALUE;
        for (double value : input) {
            if (Math.abs(value) > max) {
                max = Math.abs(value);
            }
        }
        return max;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotorEx motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("FL");
        DcMotorEx motorBackLeft = (DcMotorEx)hardwareMap.dcMotor.get("BL");
        DcMotorEx motorFrontRight =(DcMotorEx) hardwareMap.dcMotor.get("FR");
        DcMotorEx motorBackRight = (DcMotorEx)hardwareMap.dcMotor.get("BR");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = reducingDeadzone(gamepad1.left_stick_y); // Remember, this is reversed!//i reversed it again
            double x = -reducingDeadzone(gamepad1.left_stick_x * 1.1); // Counteract imperfect strafing
            boolean precisionToggle = gamepad1.right_trigger > 0.1;
            double rx = reducingDeadzone(gamepad1.right_stick_x);
            if (precisionToggle) {
                rx *= TURN_PRECISION;
            }


            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = -(Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1));
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower =(rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            double maxValue = getMax(new double[]{
                    frontLeftPower,
                    frontRightPower,
                    backLeftPower,
                    backRightPower
            });
            if (maxValue > 1) {
                frontLeftPower /= maxValue;
                frontRightPower /= maxValue;
                backLeftPower /= maxValue;
                backRightPower /= maxValue;
            }

            if (precisionToggle) {
                motorFrontLeft.setPower(frontLeftPower * PRECISIONREDUCTION);
                motorBackLeft.setPower(backLeftPower * PRECISIONREDUCTION);
                motorFrontRight.setPower(frontRightPower * PRECISIONREDUCTION);
                motorBackRight.setPower(backRightPower * PRECISIONREDUCTION);
            } else {
                motorFrontLeft.setPower(frontLeftPower);
                motorBackLeft.setPower(backLeftPower);
                motorFrontRight.setPower(frontRightPower);
                motorBackRight.setPower(backRightPower);
            }
        }

    }
    public double reducingDeadzone(double x) {
        if (x == 0) {
            return 0;
        } else if (0 < x && 0.25 > x) {
            return 0.25;
        } else if (0 > x && x > -0.25) {
            return -0.25;
        }
        return x;
    }
}