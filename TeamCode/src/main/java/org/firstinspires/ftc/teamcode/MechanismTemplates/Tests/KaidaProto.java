package org.firstinspires.ftc.teamcode.MechanismTemplates.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@Disabled
@TeleOp(name = "KaidaProto") // like Khaidi No. 150, that's crazy
public class KaidaProto extends OpMode{
    public static double power = 1;
    private DcMotorEx left; //right;

    @Override
    public void init(){
        left = (DcMotorEx) hardwareMap.dcMotor.get("L");
        //right = (DcMotorEx) hardwareMap.dcMotor.get("R");

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    @Override
    public void loop(){
        left.setPower(power);
        //right.setPower(power);

    }



}
