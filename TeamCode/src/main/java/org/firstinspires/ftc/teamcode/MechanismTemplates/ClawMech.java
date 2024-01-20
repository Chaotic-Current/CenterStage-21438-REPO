package org.firstinspires.ftc.teamcode.MechanismTemplates;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ClawMech {
    private Servo claw; //pin 2
    private Telemetry telemetry;
    public static double openPos = 0.3;
    public Gamepad gamepad;
    public static double close = 0.6;
    private SignalEdgeDetector rightBumper, rightDPadRight;
    public static double halfOpenPos = 0.45;
    public static double timeOffset = 750;
    public ElapsedTime timer;

    public boolean delayOver;

    public boolean readyForSlides;

    public enum ClawState {
        OPEN, HALF_OPEN, CLOSED, DELAY_CLOSE
    }

    private boolean bothOpenRan;

    private ClawState state;


    public ClawMech(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad) {
        this.gamepad = gamepad;
        claw = hardwareMap.get(Servo.class, "CLAW");
        state = ClawState.OPEN;
        claw.setPosition(openPos); // start position

        this.telemetry = telemetry;

//        buttonA = new SignalEdgeDetector(() -> gamepad.a);
//        buttonB = new SignalEdgeDetector(() -> gamepad.b);
        rightBumper = new SignalEdgeDetector(() -> gamepad.right_bumper);
        rightDPadRight = new SignalEdgeDetector(() -> gamepad.dpad_right);

        timer = new ElapsedTime();
    }

    public ClawMech(HardwareMap hardwareMap, Telemetry telemetry){
        claw = hardwareMap.get(Servo.class, "CLAW");
        claw.setPosition(close);
        this.telemetry = telemetry;
    }

    public void initialize() {
        claw.setPosition(halfOpenPos);
    }

    public void setBothOpen(boolean bothOpenRan){
        telemetry.addLine("Works");
        claw.setPosition(halfOpenPos);
        state = ClawState.HALF_OPEN;


        if(!bothOpenRan) {
            bothOpenRan = true;
            this.bothOpenRan = true;
            timer = new ElapsedTime();
        }
        if(timer.milliseconds() >= timeOffset){
            claw.setPosition(openPos);
            state = ClawState.OPEN;
            bothOpenRan = false;
            this.bothOpenRan = false;
        }
    }

    public Servo getClaw() {
        return claw;
    }

    public void run(IntakeMech.State e) {

        if (e == IntakeMech.State.RUNNING) {
            claw.setPosition(openPos);
            state = ClawState.OPEN;
            rightBumper.update();
            return;
        }



        if (rightDPadRight.isRisingEdge() || bothOpenRan) {
            setBothOpen(this.bothOpenRan);
        }


        if (rightBumper.isRisingEdge()) {
            if (state == ClawState.CLOSED) {
                telemetry.addData("halfway", 1);
                claw.setPosition(halfOpenPos);
                state = ClawState.HALF_OPEN;
            } else if (state == ClawState.HALF_OPEN) {
                telemetry.addData("open", 1);
                claw.setPosition(openPos);
                state = ClawState.OPEN;
            } else {
                telemetry.addData("close", 1);
                claw.setPosition(close);
                gamepad.rumble(1000);
                state = ClawState.CLOSED;
            }
        }

        telemetry.addData("Get claw position ", state.name());
        if (timer != null)
            telemetry.addData("Get timer value ", timer.milliseconds());
        rightBumper.update();
        rightDPadRight.update();
        telemetry.update();
        //delayClose();
//    buttonA.update();
//    buttonB.update();
    }

    public void close(){
        claw.setPosition(close);

        state = ClawState.CLOSED;
    }

    public void delayClose(){
        if(state.equals(ClawState.DELAY_CLOSE) && timer.milliseconds() > 500 && !delayOver) {
            readyForSlides = true;
            delayOver = true;
        }
    }

    public void open(){
        claw.setPosition(openPos);
        state= ClawState.OPEN;
    }

    public void halfOpen(){
        claw.setPosition(halfOpenPos);
    }

    public ClawState getState(){
        return state;
    }

    public void resetTimer(){
        timer.reset();
    }
}