package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class DroneThrower {
    private Servo launcher;
    public static double shot = 0.5;
    public static double loaded = 0.81;

    public DroneThrower(HardwareMap hardwareMap){
        launcher = hardwareMap.get(Servo.class, "plane");

        launcher.setPosition(loaded);
    }

    public void luanched(){
        launcher.setPosition(shot);
    }
}
