package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;



@Config
public class Intake {
    private HardwareMap hardwareMap;

    private Telemetry telemetry;

    private Gamepad gamepad1;

    private Servo intakeArmR, intakeArmL, gate;

    private DcMotorEx rollerMotor;

    private ElapsedTime timer = new ElapsedTime();



    public static double motorPow = 0.63;//0.85

    public static double intakeRestPosition = 0.7;
    public static double intakeTargetPos = 0.0;

    public static double armPos2 = 0.27;



    public static double armPos = 0.13;

    public static double flickerOpen = 0.0, flickerClose = 0.0;






    private double bottomPos = 0.302;

    private int posIndex = 0;
    public static double threshold = Integer.MAX_VALUE;
    public enum IntakeStates{
        INTAKE,
        EXTAKE,

        TOGGLE,
        PICKUP


    }
    private IntakeStates currentState = IntakeStates.PICKUP;
    private AnalogSensor IntakeCurrent;
    private Outake outake;

    public Intake(HardwareMap hw, Telemetry tele, Outake out){
        this.hardwareMap = hw;
        this.telemetry = tele;
        this.outake = out;

        rollerMotor = (DcMotorEx) hardwareMap.dcMotor.get("IN");
        rollerMotor.setDirection(DcMotorEx.Direction.REVERSE);

       // intakeArmR = hardwareMap.servo.get("inServoR");
        intakeArmL = hardwareMap.servo.get("ARM_L");
       // intakeArmR.setDirection(Servo.Direction.REVERSE);
    }

    public Intake(HardwareMap hw, Telemetry tele, Gamepad g1 , Outake outake ){
        this.hardwareMap = hw;
        this.telemetry = tele;
        this.gamepad1 = g1;
        this.outake = outake;
        //Servo Init
        ;
        //Reversing direction of right servo


        //Motor Init
        rollerMotor = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        rollerMotor.setDirection(DcMotorEx.Direction.REVERSE);
        //gate = hardwareMap.servo.get("flicker");
      //  intakeArmL = hardwareMap.get(Servo.class, "ARM_L");
       // intakeArmR = hardwareMap.servo.get("inServoR");
       // intakeArmL = hardwareMap.servo.get("inServoL");
       // intakeArmR.setDirection(Servo.Direction.REVERSE);

    }

    public void executeTeleOp(){

        //intakeArmL.setPosition((1-intakeRestPosition)*gamepad1.right_trigger+intakeRestPosition);
        //intakeArmL.setPosition(intakeRestPosition);
       // telemetry.addData("servo pos", intakeArmL.getPosition() );
       // intakeArmR.setPosition((0.5-intakeRestPosition)*gamepad1.right_trigger+intakeRestPosition+0.06);

        switch (currentState) {
            //When the roller is in the ground state, there is no movement of intake
            case PICKUP:
                //gate.setPosition(flickerClose);

                rollerMotor.setPower(0.0);
                if(timer.seconds()>0.2){
                    outake.grabTop();
                    outake.grabBottom();
                }
                //When x is pressed and the outake is at rest, the state is switched to Moving
                if(outake.getOutakeState()==Outake.OutakeStates.REST){

                    if(timer.seconds()>1.2){
                        outake.setArmReadyToPick();
                    }
                    if (gamepad1.left_trigger>0.1) {
                        currentState = IntakeStates.INTAKE;
                        outake.setArmReadyToPick();
                    }
                    else if (gamepad1.dpad_down) {
                        currentState = IntakeStates.EXTAKE;
                    }
                }
                break;

            case INTAKE:
                 rollerMotor.setPower(motorPow);
                //gate.setPosition(flickerOpen);
                outake.releaseTop();
                outake.releaseBottom();
                //if x is not held, then the state is switched back to ground
                if (!(gamepad1.left_trigger>0.1)) {
                    outake.setArmPick();
                    currentState = IntakeStates.PICKUP;
                    timer.reset();
                }
                break;

            case EXTAKE:
                rollerMotor.setPower(-motorPow);

                if (!gamepad1.dpad_down) {
                    currentState = IntakeStates.PICKUP;
                }

                break;
            }


    }
    public void executeAuto(){
        switch (currentState) {
            case PICKUP:

                rollerMotor.setPower(0.0);
                intakeArmL.setPosition(intakeRestPosition);
              //  intakeArmR.setPosition(intakeRestPosition+0.06);

                break;

            case INTAKE:

                if(true) //
                    rollerMotor.setPower(motorPow);
                else
                    rollerMotor.setPower(-motorPow);

                intakeArmL.setPosition(intakeTargetPos);
               // intakeArmR.setPosition(intakeTargetPos+0.06);
                break;

            case EXTAKE:
                rollerMotor.setPower(-motorPow);
                break;
            case TOGGLE:
                rollerMotor.setPower(motorPow);
                intakeArmL.setPosition(intakeRestPosition);
              //  outake.openClaw();
                if(timer.seconds()>0.5){
                    outake.releaseTop();
                    if(timer.seconds()>1){
                        timer.reset();
                    }
                }
                break;
        }


    }




    public IntakeStates getCurrentState(){
        return currentState;
    }

    public void setMotorPow(double n){
        motorPow = n;
    }

    public boolean canIntake(){
        return rollerMotor.getCurrent(CurrentUnit.AMPS) > threshold; // tune threshold
    }

    public void setToGround(){
       // outake.closeClawB();
       // outake.closeClaw();
        currentState = IntakeStates.PICKUP;

    }
    public void setToIntake(double pos){
        //outake.openClawB();
        //0.8 for first pixel off stack// 0.42
        //0.79 for second pixel off stack/4
        //0.77
        //0.745
        //outake.openClaw();
        intakeTargetPos = pos;
        currentState = IntakeStates.INTAKE;
    }
    public void twerk(){
        //outake.openClawB();
        //0.8 for first pixel off stack// 0.42
        //0.79 for second pixel off stack/4
        //0.77
        //0.745
      //  outake.openClaw();
        currentState = IntakeStates.TOGGLE;
    }




}