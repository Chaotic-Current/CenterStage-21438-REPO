package org.firstinspires.ftc.teamcode.Mechanisms.MechanismTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@Disabled
@TeleOp(name = "IntakeVoltageProto")
public class IntakeVoltageProto extends OpMode{
    private static double power = 1;
    private int counter;
    private DcMotorEx main;
    private double thresholdCurrent = 0.5;
    private VoltageSensor vS;

    @Override
    public void init(){
        main = (DcMotorEx) hardwareMap.dcMotor.get("main");

        vS = hardwareMap.get(VoltageSensor.class, "Control Hub");

        main.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //main.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    @Override
    public void loop(){

        main.setPower(power);

        if(main.getCurrent(CurrentUnit.AMPS) > thresholdCurrent){
            counter++;
        }


        telemetry.addData("motor voltage", main.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("counter", counter);

        telemetry.update();

    }

}
