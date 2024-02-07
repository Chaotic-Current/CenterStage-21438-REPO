package org.firstinspires.ftc.teamcode.Autos.AutoTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.ArmMecNew;

@TeleOp
@Disabled
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
