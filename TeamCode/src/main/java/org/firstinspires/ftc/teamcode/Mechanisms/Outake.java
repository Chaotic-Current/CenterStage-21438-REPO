package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@SuppressWarnings("all")
@Config
public class Outake {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private Gamepad gamepad2;
    //Change back to private later
   // public ServoImplEx armServoL;
   // public ServoImplEx armServoR;



    public ServoImplEx wristRotation;

    private PwmControl.PwmRange range;

    public Servo claw;

    public Servo wrist;
    public ClawRealign wristRelignment;

    //public ArmMecNew arms;
    private Servo armR, armL;

    public static double depositPos = 0.94, intakePos = 0.3, restPos = 0.27;

    //public Servo wristRotation;

    public Servo launcher;

    public DcMotorEx slideL;
    public DcMotorEx slideR;

    private SlideMech slideMech;
    private ArmMecNew armMec;

    public static double openPosU=0.7,openPosB = 0.825, bottom = 0.22, upper =0.25, inte = 0.3;
    public static double close = 0.2, halfOpen = 0.25, open = 0.3;





    //PID
    private static ElapsedTime slideTimer = new ElapsedTime();
    private static double lastError = 0;
    private static double integralSum =0;
    public  double Kp =0.0125;
    public  double Ki =0.0; //.00005
    public  double Kd =0.0;

    public static double Kf =0.05;

    private static double kl = 0.0;

    private static double pidOutput = 0;

    private static double lagError = 0.0;

    private static double lagOutput = 0;

    private int[] stripeLevels = {15,1500,2935};

    private int stripeIndex = 0;
    private static int pastTargetPosition = 0;

    private boolean isChanging;
    public enum OutakeStates{
        REST,

        EXTENSION,


        STACK,
        RESET,

        RIGGING,

        MANUAL
    }

    enum SlidePreference{
        LEFT,
        RIGHT,
        BOTH
    }
    public OutakeStates currentOutakeState = OutakeStates.REST;
    private Intake.IntakeStates currentIntakeState = Intake.IntakeStates.GROUND;
    private DriveTrain driveTrain;
    //Constructor for Auto
    public Outake(HardwareMap hw, Telemetry tele){
        this.hardwareMap = hw;
        this.telemetry = tele;

        //Servo Init
        /*
        wristRotation = (ServoImplEx)hardwareMap.servo.get("wristR");
        armServoL = (ServoImplEx) hardwareMap.servo.get("armServoL");
        armServoR = (ServoImplEx) hardwareMap.servo.get("armServoR"); */
        //arms = new ArmMecNew(hardwareMap);

        range = new PwmControl.PwmRange(500,2500);


      /*  armR = hardwareMap.servo.get("armR");
        armL = hardwareMap.servo.get("armL");
        armL.setDirection(Servo.Direction.REVERSE); */
       // armR.setPwmRange(range);
       // armL.setPwmRange(range);

        //clawU = hardwareMap.servo.get("clawU");
        //clawB = hardwareMap.servo.get("clawB");

        wrist = hardwareMap.servo.get("WRIST");
        //wristRelignment = new ClawRealign(hardwareMap,telemetry,wrist, driveTrain.getImu());

        // wristRotation = hardwareMap.servo.get("wristR");
        // launcher = hardwareMap.servo.get("launcher");
        // launcher.setPosition(0.5);

        claw = hardwareMap.servo.get("CLAW");

        //change directions
        // clawB.setDirection(Servo.Direction.REVERSE);

        //Motor Init
        slideMech = new SlideMech(hardwareMap);
        armMec = new ArmMecNew(hardwareMap);
    }

    //Constructor for TeleOp
    public Outake(HardwareMap hw, Telemetry tele, Gamepad g1 , DriveTrain dt){
        this.hardwareMap = hw;
        this.telemetry = tele;
        this.gamepad2 = g1;
        this.driveTrain = dt;

        //Servo Init
        range = new PwmControl.PwmRange(500,2500);

        //arms = new ArmMecNew(hardwareMap);


       // armR.setPwmRange(range);
       // armL.setPwmRange(range);


        //clawU = hardwareMap.servo.get("clawU");
        //clawB = hardwareMap.servo.get("clawB");

        wrist = hardwareMap.servo.get("WRIST");
        //wristRelignment = new ClawRealign(hardwareMap,telemetry,wrist, driveTrain.getImu());

       // wristRotation = hardwareMap.servo.get("wristR");
       // launcher = hardwareMap.servo.get("launcher");
       // launcher.setPosition(0.5);

        claw = hardwareMap.servo.get("CLAW");

        //change directions
       // clawB.setDirection(Servo.Direction.REVERSE);

        //Motor Init
        slideMech = new SlideMech(hardwareMap);
        armMec = new ArmMecNew(hardwareMap);

        /*
        slideL = (DcMotorEx) hardwareMap.dcMotor.get("slideL");
        slideR = (DcMotorEx) hardwareMap.dcMotor.get("slideR");


        //change direction later
        //slideL.setDirection(DcMotorSimple.Direction.REVERSE);
        slideR.setDirection(DcMotorSimple.Direction.REVERSE);

        slideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //disregard this if we want accurate pid ig
        slideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        */
    }
    //TeleOp Methods
    public void executeTeleOp(){
        //At any moment, you can set the target position(with dpad), and if the current state is either extension or manual, the slides would move
        //to target pos you set
        changeTargetPos();

        if(gamepad2.share){
           // launcher.setPosition(0.25);
        }

        switch (currentOutakeState) {
            case REST:

                slideMech.update();
                wrist.setPosition(0.5);
                //arms.setIntake();
                //setArmIntake();

                if(isChanging){
                    isChanging = false;
                    currentOutakeState = OutakeStates.EXTENSION;
                }

                break;


            case EXTENSION:
                //when dpad down is pressed again in extension state, outake state is changed to reset state
                if (gamepad2.dpad_down){
                    currentOutakeState = OutakeStates.RESET;
                    closeClaw();
                    slideTimer.reset();
                }
                else if(gamepad2.options && gamepad2.share) currentOutakeState = OutakeStates.RIGGING;
                else {
                    //wristRelignment.swivelToPostion(wrist, driveTrain.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),(slideTimer.seconds()>1),telemetry);
                    wrist.setPosition(0.5);
                    //when Y is pressed, clawU opens
                    if(gamepad2.y){
                       // openClawU();
                        openClaw();
                    }
                    //when X is pressed, clawB opens
                    if(gamepad2.x){
                        //openClawB();
                        halfOpenClaw();
                    }
                    if(slideMech.getCurrentTickPosition()>1200){
                        //arms.setExtake();
                        //setArmDeposit();
                        armMec.setExtake();

                    }
                    //slides extend based off PID
                    slideMech.update();

                    //if bumper inputs are given, state switches to manual and pid is interrupted
                    if (gamepad2.right_bumper || gamepad2.left_bumper) {
                        currentOutakeState = OutakeStates.MANUAL;
                        slideTimer.reset();
                    }

                }
                break;

            case MANUAL:
                if (gamepad2.dpad_down){
                    currentOutakeState = OutakeStates.RESET;
                    closeClaw();
                    slideTimer.reset();
                }
                else if(gamepad2.options && gamepad2.share) currentOutakeState = OutakeStates.RIGGING;
                else{
                    //wristRelignment.swivelToPostion(wrist, driveTrain.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),(slideTimer.seconds()>1),telemetry);
                    wrist.setPosition(0.5);
                    manual();

                    if(gamepad2.x)
                        halfOpenClaw();
                    if(gamepad2.y)
                        openClaw();
                    if(slideMech.getCurrentTickPosition()>1200)
                       // setArmDeposit();
                        armMec.setExtake();

                //if the target position of the slides were changed during manual, the state is changed back to Extension so PID
                //starts again
                    if(isChanging){
                        isChanging = false;
                        currentOutakeState = OutakeStates.EXTENSION;
                    }
                }
                break;

            case RIGGING:
                if(gamepad2.right_bumper){
                    currentOutakeState = OutakeStates.MANUAL;
                }
                else{
                    if(gamepad2.left_bumper){
                        slideMech.setPower(-0.5);
                    }
                    else{
                        slideMech.setPower(-0.4);
                    }

                }

                break;


            case RESET:
                armMec.setIntake();
                //setArmIntake();
                closeClaw();
                //claw.setPosition(0.6);
                wrist.setPosition(0.5);
                //While the slides are not at ground, the PID is being calculated
                //after the slides finally get to close to our target position, the current state changes to rest

                if (slideMech.getCurrentTickPosition() > 50) {
                    //Only starts decreasing the slide height after the arm gets most of the way down
                   if(slideTimer.seconds()>=0.8){
                       //runSlidesPID();
                       slideMech.update();
                   }
                }
                else {
                   // claw.setPosition(0.4);
                    currentOutakeState = OutakeStates.REST;
                }
                break;


        }
        telemetry.addData("",slideMech.getCurrentTickPosition());
    }
    public void executeAuto(){
       // wristRotation.setPosition(wristRPos);
        //wrist held at constant
        wrist.setPosition(0.5);
        switch(currentOutakeState){
            case EXTENSION:
                //wristRelignment.swivelToPostion(wrist, driveTrain.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),(slideTimer.seconds()>1),telemetry);
                slideMech.update();
                if(slideMech.getCurrentTickPosition()>1200){
                    armMec.setExtake();
                   // setArmDeposit();
                }
                break;

            case RESET:
                armMec.setIntake();
                //setArmIntake();
                //closeClawB();
                //claw.setPosition(0.6);
                closeClaw();
                wrist.setPosition(0.5);
                //While the slides are not at ground, the PID is being calculated
                //after the slides finally get to close to our target position, the current state changes to rest

                if (slideMech.getCurrentTickPosition() > 50) {
                    //Only starts decreasing the slide height after the arm gets most of the way down
                    if(slideTimer.seconds()>=0.8){
                        //runSlidesPID();
                        slideMech.update();
                    }
                }
                else {
                    // claw.setPosition(0.4);
                    currentOutakeState = OutakeStates.REST;
                }
                break;

            case REST:
                slideMech.update();
                wrist.setPosition(0.5);
                //setArmIntake();
                break;
        }
    }

    //needed for a check in intake
    public OutakeStates getOutakeState(){
        return currentOutakeState;
    }




    public void changeTargetPos() {
        if(gamepad2.dpad_left){
            slideMech.setLowJunction();

        }
        if(gamepad2.dpad_right) {
            slideMech.setMidJunction();

        }
        if(gamepad2.dpad_down){
            slideMech.setIntakeOrGround();

        }
        //if the target position changes,
        if(Math.abs(pastTargetPosition - slideMech.getTargetPos())>0){
            isChanging = true;
        }
        pastTargetPosition = slideMech.getTargetPos();
    }
    private void manual(){
        //slide control
        if(gamepad2.right_bumper && slideMech.getCurrentTickPosition()<3000){
            slideMech.setManualSlideUp();
        }
        else if(gamepad2.left_bumper && slideMech.getCurrentTickPosition()>0){
            slideMech.setManualSlideDown();
        }
        else{
            slideMech.setPower(Kf);
        }

    }


    //PID calculations
    private double returnPower(){
        return 0;
    }

    //AUTO METHODS

    //claw controls



    public void closeClaw(){
        claw.setPosition(close);
    }

    public void halfOpenClaw(){
        claw.setPosition(halfOpen);
    }

    public void openClaw(){
        claw.setPosition(open);
    }

    public void setTargetPosition(int targetPosition){
        this.pastTargetPosition = targetPosition;
    }
    //FSM MODIFIERS
    public void resetOutake(){
        currentOutakeState = OutakeStates.RESET;
        //pastTargetPosition = 10;
        slideMech.setIntakeOrGround();
        slideTimer.reset();
    }
    public void extendOutake(int target){
        currentOutakeState = OutakeStates.EXTENSION;
        //pastTargetPosition = target;
        slideMech.setTargetPos(target);
        slideTimer.reset();
    }

    public void extendToStack(){
        pastTargetPosition = 70;
        currentOutakeState = OutakeStates.STACK;
        slideTimer.reset();
    }
    public void setKF(double n ){
        Kf = n;
    }

    public void setClawU(double n){
        claw.setPosition(n);
    }
    public void setClawB(double n){
        claw.setPosition(n);
    }

    public void setIntake(Intake.IntakeStates i){
        currentIntakeState = i;
    }

    public void setArmIntake(){
        armR.setPosition(intakePos);
        armL.setPosition(intakePos);
    }
    public void setArmDeposit(){
        armR.setPosition(depositPos);
        armL.setPosition(depositPos);
        telemetry.addLine("Is working");
    }

    public void setArmPosition(double pos){
        armR.setPosition(pos);
        armL.setPosition(pos);
    }




}