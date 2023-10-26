package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "claw Test")
public class clawTest extends OpMode {
    private Servo claw; //pin 2
    public double openPos = 0.555;
    public double startPos = 0.3;
    public double halfOpenPos = 0.48;

    @Override
    public void init() {
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(startPos); // start position
        }

    @Override
    public void loop() {

        if(gamepad1.a){
            telemetry.addData("open", 1);
            claw.setPosition(openPos);
        }
        if(gamepad1.x){//back to start
            telemetry.addData("close", 1);
            claw.setPosition(startPos);
        }
        if(gamepad1.b){
            telemetry.addData("halfway", 1);
            claw.setPosition(halfOpenPos);
        }

        telemetry.update();

    }
}