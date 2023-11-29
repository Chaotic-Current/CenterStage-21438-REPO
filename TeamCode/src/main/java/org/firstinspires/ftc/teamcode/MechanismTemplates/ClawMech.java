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
    public static double openPos = 0.4;
    public Gamepad gamepad;
    public static double close = 0.6;
    private SignalEdgeDetector rightBumper, rightDPadRight;
    public static double halfOpenPos = 0.5;
    public static double timeOffset = 1500;
    private ElapsedTime timer;

    public enum ClawState {
        OPEN, HALF_OPEN, CLOSED
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

    public void run(IntakeMec.State e) {

        if (e == IntakeMec.State.RUNNING) {
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
//    buttonA.update();
//    buttonB.update();
    }

    public void close(){
        claw.setPosition(close);
    }

    public void open(){
        claw.setPosition(openPos);
    }
}