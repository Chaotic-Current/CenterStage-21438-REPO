package org.firstinspires.ftc.teamcode.Deprecated;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mechanisms.SignalEdgeDetector;

public class ArmMec {
    private SignalEdgeDetector dpadLeft, dpadRight;
    public static double intakePositionL, extakePositionL, intakePositionR, extakePositionR;
    private Servo armLeft, armRight;

    public ArmMec(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad){
        armLeft = hardwareMap.get(Servo.class, "EXTAKEL");
        armRight = hardwareMap.get(Servo.class,"EXTAKER");

        dpadLeft = new SignalEdgeDetector(() -> gamepad.dpad_left);
        dpadRight = new SignalEdgeDetector(() -> gamepad.dpad_right);

        intakePositionR = 1;

        extakePositionR = 0;

        intakePositionL = 0;

        extakePositionL = 1;

        armLeft.setPosition(intakePositionL);
        armRight.setPosition(intakePositionR);

    }

    public void setPositionExtake(){
        armLeft.setPosition(extakePositionL);
        armRight.setPosition(extakePositionR);
    }

    public void setPositionIntake(){
        armLeft.setPosition(intakePositionL);
        armRight.setPosition(intakePositionR);
    }


    public void run(){
        if(dpadLeft.isRisingEdge())
            setPositionExtake();
        if (dpadRight.isRisingEdge())
            setPositionIntake();

        dpadRight.update();
        dpadLeft.update();

    }

}
