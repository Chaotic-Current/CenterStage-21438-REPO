package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MechanismTemplates.ArmPID;
import org.firstinspires.ftc.teamcode.MechanismTemplates.ClawMech;
import org.firstinspires.ftc.teamcode.MechanismTemplates.IntakeMec;
import org.firstinspires.ftc.teamcode.MechanismTemplates.SignalEdgeDetector;
import org.firstinspires.ftc.teamcode.MechanismTemplates.SlideMech;

@TeleOp (name = "CS_TeleOpRobotBased")
@Config
public class DT_TeleOp extends OpMode {
    // (づ￣ 3￣)づ hellohello
    private DcMotorEx motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;
    private SlideMech slides;
    private IntakeMec intake;
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
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        wrist = hardwareMap.get(Servo.class,"WRIST");

        slides = new SlideMech(hardwareMap);

        intake = new IntakeMec(hardwareMap,telemetry,gamepad1);

        claw = new ClawMech(hardwareMap,telemetry,gamepad2);
        //claw.initialize();

        arm = new ArmPID(hardwareMap);
    }


    @Override
    public void loop(){

        if(gamePad_2_Y.isRisingEdge()){
            telemetry.addLine("Uh Oh The Bot Is Broken");
        }

        if(gamePad_2_X.isRisingEdge()){
            slides.setMidJunction();
        }

        if (gamePad_2_B.isRisingEdge()){
            slides.setLowJunction();
        }

        if(gamepad_2_A.isRisingEdge()){
            slides.setIntakeOrGround();
        }

//        if(gamepad1.dpad_left){
//            arm.setIntake();
//        }
//
//        if(gamepad1.dpad_right){
//            arm.setExtake(0.0);
//        }

        if(gamePad_2_bumperLeft.isRisingEdge() && slides.getCurrentPosition() != SlideMech.CurrentPosition.ZERO){
            arm.setExtakeOrIntake();
        }
        wrist.setPosition(wristPos);

//        if (gamepad1.right_trigger > 0.1){
//            slides.setManualSlide(400); //165 old val
//        }

//        if (gamepad1.left_trigger > 0.1){
//            slides.setManualSlide(-200);
//        }


//ytrfd
        claw.run(intake.getState());
        intake.run();
        slides.update(telemetry);
        //altDrive();
        drive();


        ElapsedTime timer = new ElapsedTime();
        arm.update(telemetry, timer);
        gamepad_2_A.update();
        gamePad_2_Y.update();
        gamePad_2_X.update();
        gamePad_2_B.update();
        gamePad_2_bumperLeft.update();
        telemetry.update();
    }

    public void altDrive(){
        telemetry.clear();
        if(gamepad1.a){
            motorFrontLeft.setPower(0.8);
            telemetry.addLine("Testing port 2");
        } else {
            motorFrontLeft.setPower(0);
        }
        if(gamepad1.b){
            motorBackLeft.setPower(0.8);
            telemetry.addLine("Testing port 0");
        } else {
            motorBackLeft.setPower(0);
        }
        if(gamepad1.y){
            motorFrontRight.setPower(0.8);
            telemetry.addLine("Testing port 1");
        } else {
            motorFrontRight.setPower(0);
        }
        if(gamepad1.x){
            motorBackRight.setPower(0.8);
            telemetry.addLine("Testing port 3");
        } else {
            motorBackRight.setPower(0);
        }
        telemetry.update();
    }

    public void drive(){
        double y = gamepad1.left_stick_y;
        double x = -reducingDeadzone(gamepad1.left_stick_x); // 👌
        boolean precisionToggle = gamepad1.right_trigger > 0.1;
        double rx = -gamepad1.right_stick_x; // 👌
        if (precisionToggle) {
            rx *= TURN_PRECESION;
        }

        /*if(1-Math.abs(x) <= 0.25)
            x = (1/Math.abs(x)) * x;*/

        // Denominator is the largest motor power (absolute value) or 1z
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        // Calculate the mecanum motor powers
        double frontLeftPower = (y + x + 2 * rx) / denominator;
        double backLeftPower = (y - x + 2 * rx) / denominator;
        double frontRightPower = (y - x - 2 * rx) / denominator;
        double backRightPower = (y + x - 2 * rx) / denominator;

        // Cube the motor powers
        frontLeftPower = Math.pow(frontLeftPower, 3);
        frontRightPower = Math.pow(frontRightPower, 3);
        backLeftPower = Math.pow(backLeftPower, 3);
        backRightPower = Math.pow(backRightPower, 3);

        // Calculate the maximum value of all the motor powers
        // The argument here is just an array separated into different lines
        double maxValue = getMax(new double[]{
                frontLeftPower,
                frontRightPower,
                backLeftPower,
                backRightPower
        });

        // Resize the motor power values
        if (maxValue > 1) {
            frontLeftPower /= maxValue;
            frontRightPower /= maxValue;
            backLeftPower /= maxValue;
            backRightPower /= maxValue;
        }
        if (precisionToggle) {
            motorFrontLeft.setPower(frontLeftPower * PRECISIONREDUCTION);
            motorBackLeft.setPower(backLeftPower * PRECISIONREDUCTION);
            motorFrontRight.setPower(frontRightPower * PRECISIONREDUCTION);
            motorBackRight.setPower(backRightPower * PRECISIONREDUCTION);
        } else {
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
        }
       /* telemetry.addData("Motor power FL", motorFrontLeft.getPower());
        telemetry.addData("Motor power BL", motorBackLeft.getPower());
        telemetry.addData("Motor power FR", motorFrontRight.getPower());
        telemetry.addData("Motor power BR", motorBackRight.getPower());*/
    }// end of drive()


    public double reducingDeadzone(double x) {
        if (x == 0) {
            return 0;
        } else if (0 < x && 0.25 > x) {
            return 0.25;
        } else if (0 > x && x > -0.25) {
            return -0.25;
        }
        return x;
    } // end of reducingDeadzone
}

//Jacob hitting that griddy on the world stage

