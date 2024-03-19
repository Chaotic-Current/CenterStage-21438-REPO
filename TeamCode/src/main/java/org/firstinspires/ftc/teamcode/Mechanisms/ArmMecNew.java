package org.firstinspires.ftc.teamcode.Mechanisms;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ArmMecNew {

    Servo leftAxon;
    Servo rightAxon;

    public static double intakePosRight = 0.01;
    public static double intakePosLeft = 0.99;
    public static double extakePosRight = 0.95;
    public static double extakePosLeft = 0.05;

    public boolean isIntake;

    public ArmMecNew(HardwareMap hardwareMap){
        leftAxon = hardwareMap.get(Servo.class, "armL");
        rightAxon = hardwareMap.get(Servo.class, "armR");
        isIntake = true;
    }

    public void togglePosition(){
        if(!isIntake){
            setIntake();
        } else {
            setExtake();
        }
        //isIntake = !isIntake;
    }

    public void setExtake(){
        leftAxon.setPosition(extakePosLeft);
        rightAxon.setPosition(extakePosRight);
        isIntake = false;
    }

    public void setIntake(){
        leftAxon.setPosition(intakePosLeft);
        rightAxon.setPosition(intakePosRight);
        isIntake = true;
    }



}
