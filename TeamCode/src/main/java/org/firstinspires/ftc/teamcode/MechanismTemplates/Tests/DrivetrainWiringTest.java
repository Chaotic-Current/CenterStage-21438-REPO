package org.firstinspires.ftc.teamcode.MechanismTemplates.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "DT_Testing")
public class DrivetrainWiringTest extends OpMode {

    private DcMotorEx motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;

    @Override
    public void init() {
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

    }

    @Override
    public void loop() {
        telemetry.clear();
        if(gamepad1.a){
            motorFrontLeft.setPower(0.8);
            telemetry.addLine("Testing port 2");
        } else {
            motorFrontLeft.setPower(0);
        }
        if(gamepad1.b){
            motorBackLeft.setPower(0.8);
            telemetry.addLine("Testing port 0");
        } else {
            motorBackLeft.setPower(0);
        }
        if(gamepad1.y){
            motorFrontRight.setPower(0.8);
            telemetry.addLine("Testing port 1");
        } else {
            motorFrontRight.setPower(0);
        }
        if(gamepad1.x){
            motorBackRight.setPower(0.8);
            telemetry.addLine("Testing port 3");
        } else {
            motorBackRight.setPower(0);
        }
        telemetry.update();
    }
}
