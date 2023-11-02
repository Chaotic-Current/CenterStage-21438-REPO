package org.firstinspires.ftc.teamcode.MechanismTemplates;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import kotlin.Suppress;

@TeleOp (name = "SlideTest")

public class SlideMotorTest extends OpMode {
    private DcMotorEx SlideRight, SlideLeft;

    @Override
    public void init() {
        SlideLeft = (DcMotorEx) hardwareMap.dcMotor.get("SL");
        SlideRight = (DcMotorEx) hardwareMap.dcMotor.get("SR");

        SlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*
        SlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        SlideRight.setDirection(DcMotorSimple.Direction.REVERSE);*/
    }

    @Override
    public void loop() {
        if(gamepad1.right_trigger > 0.14){
            SlideRight.setPower(0.25);
            SlideLeft.setPower(0.25);
        } else if(gamepad1.left_trigger > 0.14){
            SlideRight.setPower(-0.25);
            SlideLeft.setPower(-0.25);
        }
        else{
            SlideLeft.setPower(0);
            SlideRight.setPower(0);
        }

    }
}
