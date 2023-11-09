package org.firstinspires.ftc.teamcode.MechanismTemplates;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config

public class ClawMech {
    private Servo claw; //pin 2
    private Telemetry telemetry;
    public static double openPos = 0.51;
    public static double startPos = 0.32;
    private SignalEdgeDetector buttonA, buttonB, buttonX;
    public static double halfOpenPos = 0.45;



    public ClawMech(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad) {
        claw = hardwareMap.get(Servo.class, "CLAW");
        claw.setPosition(openPos); // start position

        this.telemetry = telemetry;

//        buttonA = new SignalEdgeDetector(() -> gamepad.a);
//        buttonB = new SignalEdgeDetector(() -> gamepad.b);
        buttonX = new SignalEdgeDetector(() -> gamepad.x);

        }
        public void initialize(){
            claw.setPosition(halfOpenPos);
        }

        public int toggleCount = 2;

    public void run(){
//        if(buttonA.isRisingEdge()){
//            telemetry.addData("open", 1);
//            claw.setPosition(openPos);
//        }
//        if(buttonB.isFallingEdge()){//back to start
//            telemetry.addData("close", 1);
//            claw.setPosition(startPos);
//        }
//        if(buttonX.isRisingEdge()){
//            telemetry.addData("halfway", 1);
//            claw.setPosition(halfOpenPos);
//        }

        if(buttonX.isRisingEdge()){
            if(toggleCount == 1){
                telemetry.addData("halfway", 1);
            claw.setPosition(halfOpenPos);
            toggleCount++;
            }
            else if(toggleCount == 2){
                telemetry.addData("open", 1);
            claw.setPosition(openPos);
            toggleCount++;
            }
            else{
                telemetry.addData("close", 1);
            claw.setPosition(startPos);
            toggleCount-=2;
            }
        }


    buttonX.update();
//    buttonA.update();
//    buttonB.update();
    }
}