package org.firstinspires.ftc.teamcode.MechanismTemplates;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensorMech {
    private ColorSensor color;
    private String name;

    public ColorSensorMech(HardwareMap hardwareMap, String name) {
        this.name = name;
        color = hardwareMap.get(ColorSensor.class, name);
    }

    public boolean isDetectingRed(){
        if ((color.red() > color.green()) && (color.red() > color.blue()))
            return true;
        return false;
    }

    public boolean redDeviation(double old){
        if ((color.red() > old + 20) || (color.red() < old - 20))
            return true;
        return false;
    }
    public boolean blueDeviation(double old){
        if ((color.blue() > old + 20) || (color.blue() < old - 20))
            return true;
        return false;
    }
    public boolean greenDeviation(double old){
        if ((color.green() > old + 20) || (color.green() < old - 20))
            return true;
        return false;
    }


    public double getRed(){
        return color.red();
    }
    public double getBlue(){
        return color.blue();
    }
    public double getGreen(){
        return color.green();
    }

    public void update(Telemetry telemetry){
        telemetry.addData(name + "Red", color.red());
        telemetry.addData(name + "Green", color.green());
        telemetry.addData(name +"Blue", color.blue());
        //telemetry.addData("Alpha", color.alpha());
        telemetry.update();
    }


}
