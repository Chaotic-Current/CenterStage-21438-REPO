package org.firstinspires.ftc.teamcode.MechanismTemplates;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config

public class SlideMech {

    private PIDFController slidePIDF;
    private Motor slideLeft, slideRight;

    //TODO: change values into actual tested positions instead of placeholders
    public static double HIGH_JUNCTION = 2390; //<-- 12.90V, 1970;

    public static double MID_JUNCTION = 2570; //<-- 12.90V, 1550;// old 890 as of 11/8/2023
    public static double LOW_JUNCTION = 1490;  //<-- 12.90V, 1860;
    public static double ZERO_POSITION = -50;//5V;// old val -105
    private final double MAX = 2500;
    public final static double Minimum = 0;

    public static double slideKp = 0.00065; //0.00326; //0.0039;
    public static double slideKpDown = 0.0018;
    public static double slideKpManualDown = 0.00; // 0.007 old val
    public static double slideKi = 0.00375; //0.00000325;
    public static double slideKd = 0.000000; //0.000001;
    public static double slideKf = 0.00000; //0.000069;
    public static double slideKpClimb = 0.003;
    public static double slideKpClimbDown=0.03;
    private int isClimbing = 0;
    public static double targetclimb = 3050;

    private final double[] PIDF_COFFECIENTS = {slideKp, slideKi, slideKd, slideKf};

    private double targetPos;
    double correctionLeft;
    double correctionRight;

    public enum CurrentPosition{
        ZERO,LEVEl1,LEVEL2,LEVEL3,LEVEL4,CUSTOM
    }

    private CurrentPosition currentPosition;

    public SlideMech(HardwareMap hardwareMap) {
        slideLeft = new Motor(hardwareMap, "SL", Motor.GoBILDA.RPM_312); // Pin 0 on expansion hub
        slideRight = new Motor(hardwareMap, "SR", Motor.GoBILDA.RPM_312); // Pin 1 on control hub -> pin 0 control hub

        slideLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        slideLeft.setRunMode(Motor.RunMode.VelocityControl);
        slideRight.setRunMode(Motor.RunMode.VelocityControl);

        slideLeft.setInverted(true);

        slidePIDF = new PIDFController(PIDF_COFFECIENTS[0], PIDF_COFFECIENTS[1], PIDF_COFFECIENTS[2], PIDF_COFFECIENTS[3]);

        targetPos = 0; // target position is 0 by default
        slideRight.resetEncoder();
        slideLeft.resetEncoder();
    }

    public void update(Telemetry telemetry) {
        int avg = (slideRight.getCurrentPosition() + slideLeft.getCurrentPosition()) / 2;

        if(isClimbing>0){
            slidePIDF.setPIDF(slideKpClimb,slideKi,slideKd,slideKf);
        }

        else if( avg > targetPos){
            slidePIDF.setPIDF(slideKpDown, slideKi, slideKd, slideKf);
        }else {
            slidePIDF.setPIDF(slideKp, slideKi, slideKd, slideKf);
        }
        correctionLeft = slidePIDF.calculate(slideLeft.getCurrentPosition(), targetPos);
        correctionRight = slidePIDF.calculate(slideRight.getCurrentPosition(), targetPos);

     /* telemetry.addData("targetPosition: ", targetPos);
        telemetry.addData("Right motor position: ", slideRight.getCurrentPosition());
        telemetry.addData("Left motor position: ", slideLeft.getCurrentPosition());
        telemetry.addData("Left correction: ", correctionLeft);
        telemetry.addData("Right correction: ", correctionRight);
        telemetry.update();*/



        // sets the output power of the motor
        slideLeft.set(correctionLeft);
        slideRight.set(correctionRight);
    }

    public void setIntakeOrGround() {
        targetPos = ZERO_POSITION;
        if(isClimbing>0){//was climbing up, now down
            slidePIDF.setPIDF(slideKpClimbDown,slideKi,slideKd,slideKf);
        }
        currentPosition = CurrentPosition.ZERO;
    }

    public void setLowJunction() {
        targetPos = LOW_JUNCTION;
        currentPosition = CurrentPosition.LEVEl1;
    }

    public void setMidJunction() {
        targetPos = MID_JUNCTION;
        currentPosition = CurrentPosition.LEVEL2;
    }

    public void climbUp(){
        targetPos = targetclimb;
        isClimbing = 1;
        currentPosition = CurrentPosition.CUSTOM;

    }

    public void setHighJunction() {
        targetPos = HIGH_JUNCTION;
        currentPosition = CurrentPosition.LEVEL4;
    }

    public void setManualSlide(int increment){
        double targetManual = (slideRight.getCurrentPosition() + slideLeft.getCurrentPosition()) / 2 + increment;

        if (targetManual <= MAX && targetManual >= ZERO_POSITION)
            targetPos = targetManual;
        if(increment < 0){
            slidePIDF.setPIDF(slideKpManualDown, slideKi, slideKd, slideKf);
        }
        currentPosition = CurrentPosition.CUSTOM;
    }

    public CurrentPosition getCurrentPosition(){
        return currentPosition;
    }

    public void setCustom(int custom) {
        targetPos = custom;
    }
}
