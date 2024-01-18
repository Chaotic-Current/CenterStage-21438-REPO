package org.firstinspires.ftc.teamcode.MechanismTemplates;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ArmMecNew {

    Servo leftAxon;
    Servo rightAxon;

    public static double intakePosRight = 0.015;
    public static double intakePosLeft = 0.985;
    public static double extakePosRight = 0.95;
    public static double extakePosLeft = 0.05;

    public boolean isIntake;

    public ArmMecNew(HardwareMap hardwareMap){
        leftAxon = hardwareMap.get(Servo.class, "EXTAKEL");
        rightAxon = hardwareMap.get(Servo.class, "axon");
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
//        leftAxon.setPosition(extakePosLeft);
//        rightAxon.setPosition(extakePosRight);
//        isIntake = false;
    }

    public void setIntake(){
//        leftAxon.setPosition(intakePosLeft);
//        rightAxon.setPosition(intakePosRight);
//        isIntake = true;
    }



}
