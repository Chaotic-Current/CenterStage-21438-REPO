package org.firstinspires.ftc.teamcode.MechanismTemplates;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
public class IntakeMech {
    //"MC ABHI IS ON THE REPO!!!"

    // Declaring intake motors
    private DcMotorEx intake; // pin 0
    private Telemetry telemetry;
    private static Servo left, right; // left -> 0, right -> 1
    private SignalEdgeDetector buttonY, buttonB, bumperLeft, bumperRight, dPadDown;
    private Gamepad gamepad;

    // LEFT
    public static double leftFinalPos = 0.65; //also the down position in teleOPp
    public static double leftFinalUp = 1;
    public static double leftTeleOpUp = 0.8;
    public static double leftAutoApproachPosition = .8;
    public static double leftAutoIntakePositionStage1 = .781;
    public static double leftAutoIntakePositionStage2 = .475;

    // RIGHT
    public static double rightFinalUp = 0;
    public static double rightTeleOp = 0.2;
    public static double rightAutoApproachPosition = .2;
    public static double rightAutoIntakePositionStage1 = 0.219;
    public static double rightAutoIntakePositionStage2 = 0.525;
    public static double rightFinalPos = 0.35; //also the down position in teleOPp

    // OTHER
    public static double increment = 0.025;
    public static double power = 0.75;
    public static double AutoPower = 1;
    public static double AutoExtakePower = 0.7;
    public static double ejectPower = -1.0;

    public static double threshHoldPower = 0.2;

    private double lastRecordedTime;

    private VoltageSensor vS;

    public enum State {
        RUNNING, STOPPED, REVERSE
    }

    private State state;

    private boolean isIntaking = false;

    ElapsedTime timer = new ElapsedTime();

    boolean isReduced = false;

    boolean hasRanOnce = false;

    double lastRecordedTwo;


    public IntakeMech(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad) {
        intake = (DcMotorEx) hardwareMap.dcMotor.get("IN");
        //vS = hardwareMap.get(VoltageSensor.class, "Control Hub");
        left = hardwareMap.get(Servo.class, "ARM_L");
        right = hardwareMap.get(Servo.class, "ARM_R");
        left.setPosition(0.8); // left start pos
        right.setPosition(0.2); // right start pos

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        this.telemetry = telemetry;

        this.buttonY = new SignalEdgeDetector(() -> this.gamepad.y);

        this.buttonB = new SignalEdgeDetector(() -> this.gamepad.b);

        this.gamepad = gamepad;

        this.bumperRight = new SignalEdgeDetector(() -> this.gamepad.right_bumper);
        ;

        this.bumperLeft = new SignalEdgeDetector(() -> this.gamepad.left_bumper);
    }

    public IntakeMech(HardwareMap hardwareMap, Telemetry telemetry) {
        intake = (DcMotorEx) hardwareMap.dcMotor.get("IN");
        //vS = hardwareMap.get(VoltageSensor.class, "Control Hub");
        left = hardwareMap.get(Servo.class, "ARM_L");
        right = hardwareMap.get(Servo.class, "ARM_R");

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setPosition(leftFinalPos);
        right.setPosition(rightFinalPos);

        this.telemetry = telemetry;


    }
    public IntakeMech(HardwareMap hardwareMap) {
        intake = (DcMotorEx) hardwareMap.dcMotor.get("IN");
        //vS = hardwareMap.get(VoltageSensor.class, "Control Hub");
        left = hardwareMap.get(Servo.class, "ARM_L");
        right = hardwareMap.get(Servo.class, "ARM_R");

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setPosition(leftFinalPos);
        right.setPosition(rightFinalPos);


    }

    public State getState() {
        return state;
    }

    public void start() {
        left.setPosition(leftAutoApproachPosition);
        right.setPosition(rightAutoApproachPosition);
        isIntaking = true;
        state = State.RUNNING;
        intake.setPower(AutoPower);
    }

    public void AutoIntakeServoPositionStage1(){
        left.setPosition(leftAutoIntakePositionStage1);
        right.setPosition(rightAutoIntakePositionStage1);
    }

    public void AutoIntakeServoPositionStage2(){
        left.setPosition(leftAutoIntakePositionStage2);
        right.setPosition(rightAutoIntakePositionStage2);
    }

    public void startAuto() {
        setIntake();
        state = State.RUNNING;
        intake.setPower(power);
    }

    public void reverse() {
        state = State.REVERSE;
        intake.setPower(ejectPower);
    }

    public double getIntakeVoltage() {
        return intake.getCurrent(CurrentUnit.AMPS);
    }

    public void stop() {
        setNeutral();
        state = State.STOPPED;
        intake.setPower(0);
    }

    public void run() {
        if (gamepad.left_trigger > 0.1) {
            setIntake();
            state = State.RUNNING;
            intake.setPower(power);

        } else if (gamepad.dpad_down) {
            state = State.REVERSE;
            intake.setPower(ejectPower);
            gamepad.setLedColor(255, 0, 0, 1000);

        } else {
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

        if (buttonY.isRisingEdge()) {
            telemetry.addData("yea", 1);
            left.setPosition(leftFinalPos);
            right.setPosition(rightFinalPos);
        }
        if (buttonB.isRisingEdge()) {
            telemetry.addData("idk", 1);
            left.setPosition(leftFinalUp);
            right.setPosition(rightFinalUp);
        }

        if (bumperLeft.isRisingEdge()) {
            telemetry.addData("Line up", 1);
            left.setPosition(left.getPosition() + increment);
            right.setPosition(right.getPosition() - increment);
        }

        if (bumperRight.isRisingEdge()) {
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

    public static Servo getLeft() {
        return left;
    }

    public static Servo getRight() {
        return right;
    }

    public static void setServoPower(){

    }

    public void setIntake() {
        if (!isIntaking) {
            left.setPosition(leftFinalPos);
            right.setPosition(rightFinalPos);
            isIntaking = true;
        }
    }

    public void setNeutral() {
        if (isIntaking) {
            left.setPosition(leftTeleOpUp);
            right.setPosition(rightTeleOp);
            isIntaking = false;
        }
    }

    public void setServosUp() {
        left.setPosition(leftAutoApproachPosition);
        right.setPosition(rightAutoApproachPosition);
    }

    public String getServosPos(){
        return "Left Servo Pos: " + left.getPosition() + "\nRight Servo Pos " + right.getPosition();
    }

    public void update(SampleMecanumDrive drive){
        if(!isReduced && getState() != State.STOPPED && getState() != State.REVERSE && timer.milliseconds() - lastRecordedTime > 100  && drive.getEncoder().getWheelVelocitySum() == 0){
            left.setPosition(left.getPosition()-0.01);
            right.setPosition(right.getPosition()+0.01);
            lastRecordedTime = timer.milliseconds();
        }

        telemetry.addLine("" + intake.getCurrent(CurrentUnit.AMPS));
        isReduced = intake.getCurrent(CurrentUnit.AMPS) > threshHoldPower;

        if(isReduced)
            lastRecordedTwo = timer.milliseconds();

        if(timer.milliseconds() - lastRecordedTwo > 100 && isReduced && getState() != State.STOPPED && getState() != State.REVERSE ){
            intake.setPower(0.6);
        }
    }

    //hello
}