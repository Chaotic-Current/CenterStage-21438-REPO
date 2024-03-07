package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@SuppressWarnings("all")
@Config
public class Outake {

    /*
        SERVO POSITIONS:


        !!NOTE(when grabbing pixels and moving for deposit): LIFT ARM SLIGHTLY, THEN MOVE SLIDES, THEN FLIP ARM


        (armL and armR)DEPOSIT POSITION= 0.71
        (armL and armR)BEFORE PICKING POSITION= 0.1
        (armL and armR)PICKING POSITION= 0.


        (pivot) DEFAULT = 0.5

        (wrist) INTAKE = 0.09
        (wrist) DEPOSIT = 0.64

        (tilt) DEFAULT = 0.52

        (both claws) GRAB = 0.45
        (both claws) RELEASE = 0.0




    */
    public static double armDeposit = 0.71, armReadyToPick = 0.1 ,  armPick = 0;
    public static double pivotDefault = 0.5;
    public static double wristIntake = 0.09, wristDeposit = 0.64;
    public static double tiltDefault = 0.52;
    public static double grabPos = 0.45, releasePos = 0.0;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private Gamepad gamepad2;
    //Change back to private later
   // public ServoImplEx armServoL;
   // public ServoImplEx armServoR;



    public ServoImplEx wristRotation;


    public Servo top,bottom;

    public Servo wrist,tilt,pivot;

    //public ArmMecNew arms;
    private Servo armR, armL;


    public Servo launcher;

    public DcMotorEx slideL;
    public DcMotorEx slideR;

    private SlideMech slideMech;
    private ArmMecNew armMec;






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


        wrist = hardwareMap.servo.get("WRIST");


        //change directions
        // clawB.setDirection(Servo.Direction.REVERSE);

        //Motor Init
        slideMech = new SlideMech(hardwareMap);

    }

    //Constructor for TeleOp
    public Outake(HardwareMap hw, Telemetry tele, Gamepad g1 , DriveTrain dt){
        this.hardwareMap = hw;
        this.telemetry = tele;
        this.gamepad2 = g1;
        this.driveTrain = dt;

        //Servo Init
       armR = hardwareMap.servo.get("armR");
       armL = hardwareMap.servo.get("armL");
       armL.setDirection(Servo.Direction.REVERSE);

       pivot = hardwareMap.servo.get("pivot");
       tilt = hardwareMap.servo.get("tilt");
       wrist = hardwareMap.servo.get("wrist");

       top = hardwareMap.servo.get("top");
       bottom = hardwareMap.servo.get("bottom");

      /* launcher = hardwareMap.servo.get("plane");
       launcher.setPosition(0.81); */

       slideMech = new SlideMech(hardwareMap);
    }
    //TeleOp Methods
    public void executeTeleOp(){
        //At any moment, you can set the target position(with dpad), and if the current state is either extension or manual, the slides would move
        //to target pos you set
        changeTargetPos();
        tilt.setPosition(tiltDefault);
        pivot.setPosition(pivotDefault);
        if(gamepad2.share){
            launcher.setPosition(0.5);
        }

        switch (currentOutakeState) {
            case REST:

               // slideMech.update();
                wrist.setPosition(wristIntake);
                //setArmReadyToPick();
                if(isChanging){
                    slideTimer.reset();
                    setArmReadyToPick();
                    isChanging = false;
                    currentOutakeState = OutakeStates.EXTENSION;
                }

                break;


            case EXTENSION:
                //when dpad down is pressed again in extension state, outake state is changed to reset state
                if (gamepad2.dpad_down){
                    currentOutakeState = OutakeStates.RESET;
                    releaseTop();
                    releaseBottom();
                    slideTimer.reset();
                }
                else if(gamepad2.options && gamepad2.share){ currentOutakeState = OutakeStates.RIGGING;
                }
                else {
                    //wristRelignment.swivelToPostion(wrist, driveTrain.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),(slideTimer.seconds()>1),telemetry);

                    if(gamepad2.x)
                        releaseBottom();
                    if(gamepad2.y)
                        releaseTop();
                    if(slideMech.getCurrentTickPosition()>400){
                        setArmDeposit();
                        wrist.setPosition(wristDeposit);
                    }
                    setArmDeposit();
                    wrist.setPosition(wristDeposit);
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
                    releaseTop();
                    releaseBottom();
                    slideTimer.reset();
                }
                else if(gamepad2.options && gamepad2.share){ currentOutakeState = OutakeStates.RIGGING;
                    //slideMech.climbUp();
                }
                else{
                    //wristRelignment.swivelToPostion(wrist, driveTrain.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),(slideTimer.seconds()>1),telemetry);
                    wrist.setPosition(0.5);
                    manual();

                    if(gamepad2.x)
                        releaseBottom();
                    if(gamepad2.y)
                        releaseTop();
                    if(slideMech.getCurrentTickPosition()>400){
                        setArmDeposit();
                        wrist.setPosition(wristDeposit);
                    }
                       // setArmDeposit();


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
                        slideMech.setIntakeOrGround();
                    }
                    slideMech.update();

                }

                break;


            case RESET:
                armL.setPosition(armReadyToPick);
                armR.setPosition(armReadyToPick);
                wrist.setPosition(wristIntake);
                //setArmIntake();
                releaseTop();
                //claw.setPosition(0.6);
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
                if(slideMech.getCurrentTickPosition()>120){
                    armMec.setExtake();
                   // setArmDeposit();
                }
                break;

            case RESET:
                armMec.setIntake();
                //setArmIntake();
                //closeClawB();
                //claw.setPosition(0.6);
                releaseTop();
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
        if((gamepad2.right_bumper)){
            slideMech.setManualSlideUp();
        }
        else if((gamepad2.left_bumper)){
            slideMech.setManualSlideDown();
        }
        else{
            slideMech.setPower(0.01);
        }

    }


    //PID calculations
    private double returnPower(){
        return 0;
    }

    //AUTO METHODS

    //claw controls



    public void releaseTop(){
        top.setPosition(releasePos);
    }
    public void releaseBottom(){
        bottom.setPosition(releasePos);
    }

    public void grabTop(){
        top.setPosition(grabPos);
    }

    public void grabBottom(){
        bottom.setPosition(grabPos);
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

    public void setTopPos(double n){
        top.setPosition(n);
    }
    public void setBottomPos(double n){
        bottom.setPosition(n);
    }

    public void setIntake(Intake.IntakeStates i){
        currentIntakeState = i;
    }


    public void setArmPosition(double pos){
        armR.setPosition(pos);
        armL.setPosition(pos);
    }

    public void setArmReadyToPick(){
        armR.setPosition(armReadyToPick);
        armL.setPosition(armReadyToPick);
    }
    public void setArmPick(){
        armR.setPosition(armPick);
        armL.setPosition(armPick);
    }

    public void setArmDeposit(){
        armR.setPosition(armDeposit);
        armL.setPosition(armDeposit);
    }



}