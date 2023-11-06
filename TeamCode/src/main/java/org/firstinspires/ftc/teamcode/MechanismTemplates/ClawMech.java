package org.firstinspires.ftc.teamcode.MechanismTemplates;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config

public class ClawMech {
    private Servo claw; //pin 2
    private Telemetry telemetry;
    public double openPos = 0.555;
    public double startPos = 0.32;
    private SignalEdgeDetector buttonA, buttonB, buttonX;
    public double halfOpenPos = 0.48;


    public ClawMech(HardwareMap hardwareMap, Telemetry telemetry, SignalEdgeDetector buttonA, SignalEdgeDetector buttonB, SignalEdgeDetector buttonX) {
        claw = hardwareMap.get(Servo.class, "CLAW");
        claw.setPosition(startPos); // start position

        this.telemetry = telemetry;

        this.buttonB = buttonB;
        this.buttonX = buttonX;
        this.buttonA = buttonA;
        }


    public void run(){
        if(buttonA.isRisingEdge()){
            //telemetry.addData("open", 1);
            claw.setPosition(openPos);
        }
        if(buttonX.isRisingEdge()){//back to start
            //telemetry.addData("close", 1);
            claw.setPosition(startPos);
        }
        if(buttonB.isRisingEdge()){
            //telemetry.addData("halfway", 1);
            claw.setPosition(halfOpenPos);
        }

        telemetry.update();

    }
}