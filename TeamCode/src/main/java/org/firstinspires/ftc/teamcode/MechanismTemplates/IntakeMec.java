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
    private SignalEdgeDetector buttonY,buttonB, bumperLeft, bumperRight;
    private Gamepad gamepad2;
    public static double leftFinalPos = 0.74;//moving a dist on 0.06
    public static double rightFinalPos = 0.26;
    public static double leftFinalUp = 0.9;
    public static double rightFinaUp = 0.1;


    public IntakeMec(HardwareMap hardwareMap, Telemetry telemetry, SignalEdgeDetector buttonY, SignalEdgeDetector buttonB, SignalEdgeDetector bumperRight, SignalEdgeDetector bumperLeft, Gamepad gamepad) {
        intake = (DcMotorEx) hardwareMap.dcMotor.get("IN");
        left = hardwareMap.get(Servo.class, "ARM_L");
        right = hardwareMap.get(Servo.class, "ARM_R");

        left.setPosition(0.8); // left start pos
        right.setPosition(0.2); // right start pos

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        this.telemetry = telemetry;

        this.buttonY = buttonY;

        this.buttonB = buttonB;

        this.gamepad2 = gamepad;

        this.bumperRight = bumperRight;

        this.bumperLeft = bumperLeft;
    }


    public void run() {
        if(gamepad2.right_trigger > 0.1) {
            intake.setPower(0.6);
        }else if(gamepad2.left_trigger > 0.1){
            intake.setPower(-.8);
        }else{
            intake.setPower(0);
        }
        if(buttonY.isRisingEdge()){
           telemetry.addData("yea", 1);
            left.setPosition(leftFinalPos);
            right.setPosition(rightFinalPos);
        }
        if(buttonB.isRisingEdge()){
            telemetry.addData("idk", 1);
            left.setPosition(leftFinalUp);
            right.setPosition(rightFinaUp);
        }

        if(bumperLeft.isRisingEdge()){
            telemetry.addData("Line up", 1);
            left.setPosition(left.getPosition() + 0.1);
            right.setPosition(right.getPosition() - 0.1);
        }

        if(bumperRight.isRisingEdge()){
            telemetry.addData("Line down", 1);
            left.setPosition(left.getPosition() - 0.1);
            right.setPosition(right.getPosition() + 0.1);
        }

        telemetry.update();
    }
}