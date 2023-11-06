package org.firstinspires.ftc.teamcode.MechanismTemplates.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "NoodleProto")
public class NoodleProto extends OpMode {
    public static double power = 1;
    private DcMotorEx gecko;

    @Override
    public void init(){
        gecko = (DcMotorEx) hardwareMap.dcMotor.get("gecko");
        //right = (DcMotorEx) hardwareMap.dcMotor.get("R");

        gecko.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gecko.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop(){
        gecko.setPower(power);
    }

}