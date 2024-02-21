package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensors {
    private Rev2mDistanceSensor left, right;
    private double angle= 0;

    public DistanceSensors(HardwareMap hardwareMap){
        left = hardwareMap.get(Rev2mDistanceSensor.class, "left"); // apparently i2c bus 0 is alr in use by imu so don't plug anything in there
        right = hardwareMap.get(Rev2mDistanceSensor.class, "right");
        double angle = 0;
    }
    //returns angular displacement
    public double getAngle(){
        angle = Math.atan((right.getDistance(DistanceUnit.INCH) - left.getDistance(DistanceUnit.INCH)) / 11.2);
        return angle;
    }
}
