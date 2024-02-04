package org.firstinspires.ftc.teamcode.Mechanisms.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Mechanisms.ArmPID;


public class Axon_AbsoluteServoTest extends OpMode {

    AnalogInput axonAnalogOutput;
    CRServo axon;

    ArmPID arm;

    double correctedAngle;




    @Override
    public void init(){
        //axon = hardwareMap.get(CRServo.class, "axon");
        axonAnalogOutput = hardwareMap.analogInput.get("axonSensor");

        //arm = new ArmPID(hardwareMap);


    }




    @Override
    public void loop() {
/*
        if(hardwareMap.analogInput.get("axonSensor").getVoltage() < 0.8){
            correctedAngle =  Math.abs(axonAnalogOutput.getVoltage() - 0.8)/3.3 * 360;
        } else{
            correctedAngle =  (Math.abs(axonAnalogOutput.getVoltage() - 3.3)/3.3 * 360 + (0.8/3.3 * 360));
        }
        telemetry.addData("Servo Voltage : ", axonAnalogOutput.getVoltage());
        telemetry.addData("Servo Position : ", axonAnalogOutput.getVoltage()/3.3 * 360);
        telemetry.addData("Corrected Position", correctedAngle);
*/
        telemetry.addData("output ", axonAnalogOutput.getVoltage());
        telemetry.update();
    }

}


