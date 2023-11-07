package org.firstinspires.ftc.teamcode.MechanismTemplates.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "AbsoluteServoTest")
public class Axon_AbsoluteServoTest extends OpMode {

    AnalogInput axonAnalogOutput;
    CRServo axon;






    @Override
    public void init(){
        axon = hardwareMap.get(CRServo.class, "axon");
        axonAnalogOutput = hardwareMap.analogInput.get("axonSensor");


    }


    @Override
    public void loop() {
        telemetry.addData("Servo Voltage : ", axonAnalogOutput.getVoltage());
        telemetry.addData("Servo Position : ", axonAnalogOutput.getVoltage()/3.3 * 360);


            telemetry.addData("Running",1);
            axon.setPower(0.2);



        telemetry.update();
    }

}


