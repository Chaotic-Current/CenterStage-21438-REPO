package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Intake Test")
public class IntakeTest extends OpMode {
    //"MC ABHI IS ON THE REPO!!!"

    // Declaring intake motors
    private DcMotorEx intake;

    private Servo left, right;

    @Override
    public void init() {
        intake = (DcMotorEx) hardwareMap.dcMotor.get("FL"); // Pin 2 -> pin 3

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Running without an encoder allows us to plug in a raw value rather than one that is proportional

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse motors
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        // We want to check this every time the loop runs
        drive();
    }

    public void drive(){
        double y = reducingDeadzone(gamepad1.left_stick_y); // Remember, this is reversed!
        double x = reducingDeadzone(-gamepad1.left_stick_x);
        boolean precisionToggle = gamepad1.right_trigger > 0.1;
        double rx = reducingDeadzone(-gamepad1.right_stick_x * 0.75);
        if (precisionToggle) {
            rx *= TURN_PRECESION;
        }

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        // Calculate the mecanum motor powers
        double frontLeftPower = (y + x + 2 * rx) / denominator;
        double backLeftPower = (y - x + 2 * rx) / denominator;
        double frontRightPower = (y - x - 2 * rx) / denominator;
        double backRightPower = (y + x - 2 * rx) / denominator;

        // Cube the motor powers
        frontLeftPower = Math.pow(frontLeftPower, 3);
        frontRightPower = Math.pow(frontRightPower, 3);
        backLeftPower = Math.pow(backLeftPower, 3);
        backRightPower = Math.pow(backRightPower, 3);

        // Calculate the maximum value of all the motor powers
        // The argument here is just an array separated into different lines
        double maxValue = getMax(new double[]{
                frontLeftPower,
                frontRightPower,
                backLeftPower,
                backRightPower
        });
        // Resize the motor power values
        if (maxValue > 1) {
            frontLeftPower /= maxValue;
            frontRightPower /= maxValue;
            backLeftPower /= maxValue;
            backRightPower /= maxValue;
        }

        telemetry.addData("frontLeftPow", frontLeftPower);
        telemetry.addData("frontRightPow", frontRightPower);
        telemetry.addData("backLeftPow", backLeftPower);
        telemetry.addData("backRightPow", backRightPower);
        telemetry.update();

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

    }// end of drive()

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



