package org.firstinspires.ftc.teamcode.Mechanisms.MechanismTests;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class REV2mArrayMech{
    Rev2mDistanceSensor left, right;
    private IMU imu;
    private double angle;


    public REV2mArrayMech(HardwareMap hw) {
        left = hw.get(Rev2mDistanceSensor.class, "left"); // apparently i2c bus 0 is alr in use by imu so don't plug anything in there
        right = hw.get(Rev2mDistanceSensor.class, "right");
        double angle = 0;
    }

    public double execute(){
        return Math.atan((right.getDistance(DistanceUnit.INCH)-left.getDistance(DistanceUnit.INCH)) / 11.2);
    }



}
