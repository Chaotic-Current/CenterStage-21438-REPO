package org.firstinspires.ftc.teamcode.Mechanisms.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config

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
