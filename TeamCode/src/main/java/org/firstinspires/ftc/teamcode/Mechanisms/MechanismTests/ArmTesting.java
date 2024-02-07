package org.firstinspires.ftc.teamcode.Mechanisms.MechanismTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.ArmMecNew;
import org.firstinspires.ftc.teamcode.Mechanisms.SignalEdgeDetector;
@Disabled
@TeleOp

public class ArmTesting extends OpMode {
    private ArmMecNew arm;
    SignalEdgeDetector GamePad_2_DpadLeft = new SignalEdgeDetector(() -> gamepad2.dpad_left);

    @Override
    public void init() {
        arm = new ArmMecNew(hardwareMap);
    }

    @Override
    public void loop() {

        if(GamePad_2_DpadLeft.isRisingEdge()){
            arm.togglePosition();
        }

        GamePad_2_DpadLeft.update();

    }
}
