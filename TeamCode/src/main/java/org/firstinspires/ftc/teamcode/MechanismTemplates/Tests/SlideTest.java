package org.firstinspires.ftc.teamcode.MechanismTemplates.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "SlideTest")
public class SlideTest extends OpMode {
    //"MC ABHI IS ON THE REPO!!!"

    // Declaring intake motors
    private DcMotorEx SR, SL; // pin 0

    @Override
    public void init() {
        SR = (DcMotorEx) hardwareMap.dcMotor.get("SR");
        SL = (DcMotorEx) hardwareMap.dcMotor.get("SL");

        SR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SR.setDirection(DcMotorSimple.Direction.REVERSE);
       // SL.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {//-================================================================================================================================================================================================================================================
        if(gamepad1.right_trigger > 0.2){
            SR.setPower(1);
            SL.setPower(1);
        }else if(gamepad1.left_trigger > 0.2){
            SR.setPower(-1);
            SL.setPower(-1);
        }
        else{
            SR.setPower(0);
            SL.setPower(0);
        }


    }
}



