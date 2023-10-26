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
    private DcMotorEx intake; // pin 0

    private Servo left, right; // left -> 0, right -> 1
    public static double leftFinalPos = 0.74;//moving a dist on 0.06
    public static double rightFinalPos = 0.26;

    @Override
    public void init() {
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");

        left.setPosition(0.8); // left start pos
        right.setPosition(0.2); // right start pos

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if(gamepad1.right_trigger > 0.1) {
            intake.setPower(1);
        }else{
            intake.setPower(0);
        }
        if(gamepad1.y){
            telemetry.addData("yea", 1);
            left.setPosition(leftFinalPos);
            right.setPosition(rightFinalPos);
        }
        if(gamepad1.b){
            telemetry.addData("idk", 1);
            left.setPosition(0.8);
            right.setPosition(0.2);
        }

        telemetry.update();

    }
}