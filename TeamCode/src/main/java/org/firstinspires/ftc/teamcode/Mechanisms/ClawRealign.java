package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mechanisms.SignalEdgeDetector;
import org.firstinspires.ftc.teamcode.Mechanisms.SlideMech;

public class ClawRealign {
    public static double servoPosition = 0.5;
    //private Servo servo; // at pin 0 on control hub
    //private IMU imu;
    private double botHeading;
    private Telemetry telemetry;
    private SignalEdgeDetector Gamepad1_Options;

    public ClawRealign(HardwareMap hardwareMap, Telemetry telemetry, Servo servo, IMU imu){
        servo =hardwareMap.get(Servo.class, "WRIST");
        servo.setPosition(servoPosition);

        // Retrieve the IMU from the hardware map

        this.telemetry = telemetry;
    }

    public void swivelToPostion( Servo servo, double botHeading, boolean canRealign, Telemetry telemetry)
    {
            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.

         if(!canRealign) {
            telemetry.addLine("Not changing heading");
            servo.setPosition(0.5);

        }
        else{
            telemetry.addData("Bot Heading: ", botHeading);
            servo.setPosition(0.5 - (botHeading/180));
        }

    }
}