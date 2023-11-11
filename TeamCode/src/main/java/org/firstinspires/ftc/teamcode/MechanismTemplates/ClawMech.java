package org.firstinspires.ftc.teamcode.MechanismTemplates;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config

public class ClawMech {
    private Servo claw; //pin 2
    private Telemetry telemetry;
    public static double openPos = 0.52;
    public Gamepad gamepad;
    public static double close = 0.32;
    private SignalEdgeDetector rightBumper;
    public static double halfOpenPos = 0.45;



    public ClawMech(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad) {
        this.gamepad = gamepad;
        claw = hardwareMap.get(Servo.class, "CLAW");
        claw.setPosition(openPos); // start position

        this.telemetry = telemetry;

//        buttonA = new SignalEdgeDetector(() -> gamepad.a);
//        buttonB = new SignalEdgeDetector(() -> gamepad.b);
          rightBumper = new SignalEdgeDetector(() -> gamepad.right_bumper);

        }
        public void initialize(){
            claw.setPosition(halfOpenPos);
        }

        public int toggleCount = 2;

    public void run(IntakeMec.State e){
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

        if(e == IntakeMec.State.RUNNING){
            claw.setPosition(openPos);
            toggleCount = 1;
            rightBumper.update();
            return;
        }


        if(rightBumper.isRisingEdge()){
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
            claw.setPosition(close);
            gamepad.rumble(1000);
            toggleCount-=2;
            }
        }


    rightBumper.update();
//    buttonA.update();
//    buttonB.update();
    }
}