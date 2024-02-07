package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Mechanisms.ClawMech;
import org.firstinspires.ftc.teamcode.Mechanisms.DroneThrower;
import org.firstinspires.ftc.teamcode.Mechanisms.ArmMecNew;
import org.firstinspires.ftc.teamcode.Mechanisms.IntakeMech;
import org.firstinspires.ftc.teamcode.Mechanisms.SignalEdgeDetector;
import org.firstinspires.ftc.teamcode.Mechanisms.SlideMech;
import org.firstinspires.ftc.teamcode.Mechanisms.MechanismTests.ClawRealign;

@TeleOp (name = "CS_TeleOpRobotBased")
@Config
public class V1TeleOp extends OpMode {
    // (づ￣ 3￣)づ hellohello
    private DcMotorEx motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;
    private SlideMech slides;
    private IntakeMech intake;
    private ClawMech claw;
    private Servo wrist;
    ClawRealign cr;
    private IMU imu;
    private double botHeading;

    private boolean canRealign = false;

    //private Servo planeLaucher;
    //private ArmPID arm;
    private ArmMecNew arm;
    private DroneThrower thrower;
    private boolean timerIsready = false;
    SignalEdgeDetector gamepad_2_A = new SignalEdgeDetector(() -> gamepad2.a);
    SignalEdgeDetector gamePad_2_Y = new SignalEdgeDetector(() -> gamepad2.y);
    SignalEdgeDetector gamePad_2_X = new SignalEdgeDetector(() -> gamepad2.x);
    SignalEdgeDetector gamePad_2_B = new SignalEdgeDetector(() -> gamepad2.b);
    SignalEdgeDetector gamePad_2_bumperLeft = new SignalEdgeDetector(() -> gamepad2.left_bumper);
    SignalEdgeDetector gamePad_1_DpadUp = new SignalEdgeDetector(() -> gamepad1.dpad_up);
    SignalEdgeDetector gamePad_1_DpadDown = new SignalEdgeDetector(() -> gamepad1.dpad_down);
    SignalEdgeDetector gamePad_1_DpadLeft = new SignalEdgeDetector(() -> gamepad1.dpad_left);
    SignalEdgeDetector GamePad_2_DpadLeft = new SignalEdgeDetector(() -> gamepad2.dpad_left);

    SignalEdgeDetector GamePad_2_DpadUp = new SignalEdgeDetector(() -> gamepad2.dpad_up);
    public static double wristPos = 0.5;
    private final double PRECISIONREDUCTION = 0.39;
    private final double TURN_PRECESION = 0.65;

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime timer2;
    private boolean slidesUp = false;
    private boolean slidesDown = false;

    private boolean isGoingUp = false;

    public static double clawDelay = 400;
    public static int armDelay = 400;

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
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        wrist = hardwareMap.get(Servo.class,"WRIST");
        wrist.setPosition(0.5);

        //planeLaucher = hardwareMap.get(Servo.class,"plane");

        slides = new SlideMech(hardwareMap);


        intake = new IntakeMech(hardwareMap,telemetry,gamepad1);

        claw = new ClawMech(hardwareMap,telemetry,gamepad2);
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        //claw.initialize();
        cr = new ClawRealign(hardwareMap, telemetry, wrist, imu);
        arm = new ArmMecNew(hardwareMap);
        arm.setIntake();

        thrower = new DroneThrower(hardwareMap);
    }


    @Override
    public void loop(){
        if(gamePad_2_Y.isRisingEdge()){
            claw.close();
            isGoingUp = true;
            timer.reset();
            slides.setTargetPosQueued(SlideMech.CurrentPosition.LEVEL3);
            if(slides.isUp(telemetry)){
                slides.setHighJunction();
            }
        }

        if(gamePad_1_DpadUp.isRisingEdge()){
            claw.close();
            isGoingUp = true;
            timer.reset();
            slides.setTargetPosQueued(SlideMech.CurrentPosition.CLIMB);
            if(slides.isUp(telemetry)){
                slides.climbUp();
            }
        }

        if(gamePad_2_X.isRisingEdge()){
            claw.close();
            isGoingUp = true;
            timer.reset();
            slides.setTargetPosQueued(SlideMech.CurrentPosition.LEVEL2);
            if(slides.isUp(telemetry)){
                slides.setMidJunction();
            }
        }

        if (gamePad_2_B.isRisingEdge()){
            claw.close();
            isGoingUp = true;
            timer.reset();
            slides.setTargetPosQueued(SlideMech.CurrentPosition.LEVEl1);
            if(slides.isUp(telemetry)){
                slides.setLowJunction();
            }
        }

        if(slides.isUp(telemetry)){
            isGoingUp = false;
            telemetry.addLine("setFalse");
        }

        // TARGET_POS QUEUE :
        // used to set slide pos after a delay and keep everything in for the slides in one area
        if(isGoingUp && timer.milliseconds() > clawDelay){
            if(slides.targetPosQueued.equals(SlideMech.CurrentPosition.LEVEl1)){
                slides.setLowJunction();
            } else if (slides.targetPosQueued.equals(SlideMech.CurrentPosition.LEVEL2)){
                slides.setMidJunction();
            } else if (slides.targetPosQueued.equals(SlideMech.CurrentPosition.LEVEL3)){
                slides.setHighJunction();
            } else if (slides.targetPosQueued.equals(SlideMech.CurrentPosition.CLIMB)) {
                slides.climbUp();
            } else {
                slides.setLowJunction();
            }

            slidesUp = true;
            slidesDown = false;
            timer.reset();
        }

        if(slidesUp){
            if(timer.milliseconds() >= armDelay) {
                if(!timerIsready){
                    timer2 =new ElapsedTime();
                    timerIsready = true;
                }
                arm.setExtake();
                if(timer2 != null && timer2.milliseconds() >1000){
                    canRealign= true;
                    telemetry.addLine("Is ready to rotate");
                }
                timer.reset();
            }
        }
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        telemetry.addData("isGoingUp", isGoingUp);
        telemetry.addData("slides.isUp()", slides.isUp(telemetry));
        telemetry.addData("timer", timer.milliseconds());
        telemetry.addData("slidesUp", slidesUp);
        telemetry.addData("left servo position: ", IntakeMech.getLeft().getPosition());
        telemetry.addData("right servo position: ", IntakeMech.getRight().getPosition());
        if(gamepad_2_A.isRisingEdge()){

            claw.close();

            if(slides.isClimbing) {
                slides.setIntakeOrGround();
            }
            timer.reset();
            if(!slides.isClimbing()){
                arm.setIntake();
            }
            slidesDown = true;
            slidesUp= false;
        }


        if(slidesDown){
            if(timer.milliseconds() >= 1000   && !slides.isClimbing){
                slides.setIntakeOrGround();
                timer.reset();
            }
            if(timer2 != null){
                timer2 = null;
                timerIsready = false;
                canRealign=false;
            }
        }


        if(GamePad_2_DpadUp.isRisingEdge()){
            slides.setIntakeOrGround();
        }

        if(gamePad_1_DpadLeft.isRisingEdge()){
            thrower.luanched();
            telemetry.addData("works",0);
        }


        //wrist.setPosition(wristPos);

        if (gamepad2.right_trigger > 0.1){
            slides.setManualSlideUp(); //165 old val
        }else if (gamepad2.left_trigger > 0.1){
            slides.setManualSlideDown();
        }else{
            slides.setCurrentPosition(SlideMech.CurrentPosition.LEVEl1);
        }


        claw.run(intake.getState());
        intake.run();
        slides.update(telemetry);
        drive();
        ElapsedTime timer = new ElapsedTime();

        //arm.update(telemetry, timer);
        gamepad_2_A.update();
        gamePad_2_Y.update();
        gamePad_2_X.update();
        gamePad_2_B.update();
        gamePad_1_DpadLeft.update();
        gamePad_1_DpadUp.update();
        gamePad_1_DpadDown.update();
        gamePad_2_bumperLeft.update();
        GamePad_2_DpadUp.update();
        cr.swivelToPostion(slides.getCurrentPosition(), wrist, botHeading, canRealign, telemetry);



        GamePad_2_DpadLeft.update();
        //telemetry.addLine("Position : " + arm.getArmPosition());
        telemetry.update();
    }


    public void drive(){
        double y = -gamepad1.left_stick_y;
        double x = reducingDeadzone(gamepad1.left_stick_x); // 👌
        boolean precisionToggle = gamepad1.right_trigger > 0.1;
        double rx = gamepad1.right_stick_x; // 👌
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

