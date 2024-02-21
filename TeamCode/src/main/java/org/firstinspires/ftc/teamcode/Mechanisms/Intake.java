package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


public class Intake {
    private HardwareMap hardwareMap;

    private Telemetry telemetry;

    private Gamepad gamepad1;

    private Servo intakeServoL;

    private Servo intakeServoR;

    private DcMotorEx rollerMotor;

    private ElapsedTime timer = new ElapsedTime();



    private double motorPow = 0.85;

    private double armPos = 0.0;


    private double bottomPos = 0.302;

    private int posIndex = 0;
    private double threshold;
    public enum IntakeStates{
        INTAKE,
        EXTAKE,

        TOGGLE,
        GROUND


    }
    private IntakeStates currentState = IntakeStates.GROUND;
    private AnalogSensor IntakeCurrent;
    private Outake outake;

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

    }

    public void execute(){


        switch (currentState) {
            //When the roller is in the ground state, there is no movement of intake
            case GROUND:
                rollerMotor.setPower(0.0);
                //When x is pressed and the outake is at rest, the state is switched to Moving
                if (gamepad1.x) {
                    currentState = IntakeStates.INTAKE;
                        /*
                    outake.openClawB();
                    outake.openClawU(); */
                }
                else if (gamepad1.b) {
                    currentState = IntakeStates.EXTAKE;
                        /*
                    outake.openClawB();
                    outake.openClawU(); */
                }
                else if(gamepad1.y){
                    timer.reset();
                    currentState = IntakeStates.TOGGLE;
                        /*
                    outake.openClawB();
                    outake.openClawU();*/

                }

                break;

            case INTAKE:

                if(!canIntake())
                    rollerMotor.setPower(motorPow);
                else
                    rollerMotor.setPower(-motorPow);

                //if x is not held, then the state is switched back to ground
                if (!gamepad1.x) {
                        /*
                    outake.closeClawB();
                outake.closeClawU(); */
                    currentState = IntakeStates.GROUND;
                }
                break;

            case EXTAKE:
                rollerMotor.setPower(-motorPow);

                if (!gamepad1.b) {

                  //  outake.closeClawB();
                  //  outake.closeClawU();
                    currentState = IntakeStates.GROUND;
                }

                break;
            case TOGGLE:
                if(timer.seconds()<0.3){
                    rollerMotor.setPower(-0.7);
                }
                else{
                    rollerMotor.setPower(motorPow);
                }
                if(timer.seconds()>=1){
                    currentState = IntakeStates.GROUND;
                   // outake.closeClawB();
                   // outake.closeClawU();
                }



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




}