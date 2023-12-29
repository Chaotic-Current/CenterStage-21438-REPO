package org.firstinspires.ftc.teamcode.MechanismTemplates.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MechanismTemplates.IntakeMech;

import javax.crypto.spec.OAEPParameterSpec;

@TeleOp
public class AutoIntakeTest extends OpMode {
private IntakeMech intake;
private ElapsedTime timer;
    int counter = 0;
    @Override
    public void init() {
        intake = new IntakeMech(hardwareMap);
        timer = new ElapsedTime();
    }

    @Override
    public void loop() {
        timer.reset();



        if(gamepad1.right_trigger >0.4)
        {
            intake.start();
            if(intake.getIntakeVoltage()>0.5){
                counter++;
                telemetry.addData("voltage", intake.getIntakeVoltage());
            }
            if(counter>2){
                intake.reverse();
            }
        }

        if(gamepad1.left_trigger > 0.4)
            counter = 0;

    }
}
