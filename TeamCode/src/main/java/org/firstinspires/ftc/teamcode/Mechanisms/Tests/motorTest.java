package org.firstinspires.ftc.teamcode.Mechanisms.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Mechanisms.ArmPID;
import org.firstinspires.ftc.teamcode.Mechanisms.ClawMech;
import org.firstinspires.ftc.teamcode.Mechanisms.IntakeMech;
import org.firstinspires.ftc.teamcode.Mechanisms.SignalEdgeDetector;
import org.firstinspires.ftc.teamcode.Mechanisms.SlideMech;


public class motorTest extends OpMode {
    // (づ￣ 3￣)づ hellohello
    private DcMotorEx motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;
    private SlideMech slides;
    private IntakeMech intake;
    private ClawMech claw;
    private Servo wrist;
    private ArmPID arm;
    SignalEdgeDetector gamepad_2_A = new SignalEdgeDetector(() -> gamepad2.a);
    SignalEdgeDetector gamePad_2_Y = new SignalEdgeDetector(() -> gamepad2.y);
    SignalEdgeDetector gamePad_2_X = new SignalEdgeDetector(() -> gamepad2.x);
    SignalEdgeDetector gamePad_2_B = new SignalEdgeDetector(() -> gamepad2.b);
    SignalEdgeDetector gamePad_2_bumperLeft = new SignalEdgeDetector(() -> gamepad2.left_bumper);
    public static double wristPos = 0.5;
    private final double PRECISIONREDUCTION = 0.39;
    private final double TURN_PRECESION = 0.65;

    /**
     * Get the maximum absolute value from a static array of doubles
     *
     * @param input the input array of double values
     * @return the maximum value from the input array
     */
    private double getMax(double[] input) {
        double max = Integer.MIN_VALUE;
        for (double value : input) {
            if (Math.abs(value) > max) {
                max = Math.abs(value);
            }
        }
        return max;
    }


    @Override
    public void init(){
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("FL"); // Pin 0
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("BL"); // Pin 1
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("FR"); // Pin 2
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("BR"); // Pin 3

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Reverse motors
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        wrist = hardwareMap.get(Servo.class,"WRIST");

        slides = new SlideMech(hardwareMap);

        intake = new IntakeMech(hardwareMap,telemetry,gamepad1);

        claw = new ClawMech(hardwareMap,telemetry,gamepad2);
        //claw.initialize();

        arm = new ArmPID(hardwareMap);
    }


    @Override
    public void loop(){


    }
}