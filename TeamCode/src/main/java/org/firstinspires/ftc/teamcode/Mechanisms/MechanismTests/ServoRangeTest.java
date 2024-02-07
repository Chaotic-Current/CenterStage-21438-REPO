package org.firstinspires.ftc.teamcode.Mechanisms.MechanismTests;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class ServoRangeTest extends OpMode{
    private Servo servo1;
    public static double val = 0.5;
    @Override
    public void init() {
        servo1 = hardwareMap.get(Servo.class, "CLAW");

    }
    @Override
    public void loop() {

        if(gamepad1.b){
            servo1.setPosition(val);
        }
        if(gamepad1.a){
            servo1.setPosition(0);
        }

    }
}
