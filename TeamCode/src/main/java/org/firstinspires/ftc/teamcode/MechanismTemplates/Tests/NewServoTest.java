package org.firstinspires.ftc.teamcode.MechanismTemplates.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "NewServoTest")
public class NewServoTest extends OpMode {
    public static double pos = 0.5;
    private Servo servo;

    @Override
    public void init(){
        Servo servo = hardwareMap.get(Servo.class, "servo");
    }

    @Override
    public void loop(){
        servo.setPosition(pos);

    }

}
