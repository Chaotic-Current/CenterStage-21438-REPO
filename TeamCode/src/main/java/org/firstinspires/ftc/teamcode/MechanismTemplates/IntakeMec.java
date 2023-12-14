package org.firstinspires.ftc.teamcode.MechanismTemplates;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class IntakeMec  {
    //"MC ABHI IS ON THE REPO!!!"

    // Declaring intake motors
    private DcMotorEx intake; // pin 0
    private Telemetry telemetry;
    private Servo left, right; // left -> 0, right -> 1
    private SignalEdgeDetector buttonY,buttonB, bumperLeft, bumperRight, dPadDown;
    private Gamepad gamepad;
    public static double leftFinalPos = 0.63;//moving a dist on 0.06
    public static double rightFinalPos = 0.37;
    public static double increment = 0.025;
    public static double leftFinalUp = 0.9;
    public static double rightFinalUp = 0.1;

    public static double power = 0.75;

    public static double ejectPower = -1.0;

    public enum State{
        RUNNING,STOPPED, REVERSE
    }

    private State state;

    private boolean isIntaking = false;


    public IntakeMec(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad) {
        intake = (DcMotorEx) hardwareMap.dcMotor.get("IN");
        left = hardwareMap.get(Servo.class, "ARM_L");
        right = hardwareMap.get(Servo.class, "ARM_R");
        left.setPosition(0.8); // left start pos
        right.setPosition(0.2); // right start pos

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        this.telemetry = telemetry;

        this.buttonY =  new SignalEdgeDetector(() -> this.gamepad.y);

        this.buttonB = new SignalEdgeDetector(()-> this.gamepad.b);

        this.gamepad = gamepad;

        this.bumperRight = new SignalEdgeDetector(() -> this.gamepad.right_bumper);;

        this.bumperLeft = new SignalEdgeDetector(() -> this.gamepad.left_bumper);
    }

    public IntakeMec(HardwareMap hardwareMap){
        intake = (DcMotorEx) hardwareMap.dcMotor.get("IN");
        left = hardwareMap.get(Servo.class, "ARM_L");
        right = hardwareMap.get(Servo.class, "ARM_R");

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setPosition(0.8);
        left.setPosition(0.2);
    }

    public State getState(){
        return state;
    }

    public void run() {
        if(gamepad.left_trigger > 0.1) {
            setIntake();
            state = State.RUNNING;
            intake.setPower(power);

        } else if(gamepad.dpad_down){
            state = State.REVERSE;
            intake.setPower(ejectPower);
            gamepad.setLedColor(255, 0, 0, 1000);

        } else{
            setNeutral();
            state = State.STOPPED;
            intake.setPower(0);
        }

        /*
        if(state == State.RUNNING){
            left.setPosition(leftFinalPos);
            right.setPosition(rightFinalPos);
        }
         */

        if(buttonY.isRisingEdge()){
           telemetry.addData("yea", 1);
            left.setPosition(leftFinalPos);
            right.setPosition(rightFinalPos);
        }
        if(buttonB.isRisingEdge()){
            telemetry.addData("idk", 1);
            left.setPosition(leftFinalUp);
            right.setPosition(rightFinalUp);
        }

        if(bumperLeft.isRisingEdge()){
            telemetry.addData("Line up", 1);
            left.setPosition(left.getPosition() + increment);
            right.setPosition(right.getPosition() - increment);
        }

        if(bumperRight.isRisingEdge()){
            telemetry.addData("Line down", 1);
            left.setPosition(left.getPosition() - increment);
            right.setPosition(right.getPosition() + increment);
        }

        telemetry.update();
        buttonB.update();
        buttonY.update();
        bumperLeft.update();
        bumperRight.update();
    }

    public DcMotorEx getIntake() {
        return intake;
    }

    public Servo getLeft() {
        return left;
    }

    public Servo getRight() {
        return right;
    }

    public void setIntake(){
        if(!isIntaking) {
            left.setPosition(leftFinalPos);
            right.setPosition(rightFinalPos);
            isIntaking = true;
        }
    }

    public void setNeutral(){
        if(isIntaking) {
            left.setPosition(leftFinalUp);
            right.setPosition(rightFinalUp);
            isIntaking = false;
        }
    }
}