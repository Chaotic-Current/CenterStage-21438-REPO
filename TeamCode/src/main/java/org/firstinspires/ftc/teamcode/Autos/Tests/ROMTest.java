package org.firstinspires.ftc.teamcode.Autos.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MechanismTemplates.ArmMecNew;

@TeleOp
public class ROMTest extends OpMode {

   ArmMecNew arm;
    @Override
    public void init() {
      arm = new ArmMecNew(hardwareMap);
      arm.setIntake();

    }

    @Override
    public void loop() {
        if(gamepad1.a)
           arm.setExtake();

        if(gamepad1.b)
            arm.setIntake();

    }
}
