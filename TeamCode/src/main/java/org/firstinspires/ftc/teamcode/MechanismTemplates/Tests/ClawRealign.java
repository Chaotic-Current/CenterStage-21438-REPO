package org.firstinspires.ftc.teamcode.MechanismTemplates.Tests;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MechanismTemplates.SignalEdgeDetector;
import org.firstinspires.ftc.teamcode.MechanismTemplates.SlideMech;

@Config

public class ClawRealign {
    public static double servoPosition = 0.5;
    private Servo servo; // at pin 0 on control hub
    private IMU imu;
    private double botHeading;
    private Telemetry telemetry;
    private SignalEdgeDetector Gamepad1_Options;


    public ClawRealign(HardwareMap hardwareMap, Telemetry telemetry, SignalEdgeDetector Gamepad1_Options){
        servo =hardwareMap.get(Servo.class, "servo");
        servo.setPosition(servoPosition);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");

        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        this.telemetry = telemetry;
    }


    public void swivelToPostion(SlideMech.CurrentPosition e, int currentMotorPosition)
    {
        if(e == SlideMech.CurrentPosition.ZERO)
            return;
            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.options) {
                imu.resetYaw();
        }

             botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        telemetry.update();
        telemetry.addData("Bot Heading: ", botHeading);

        servo.setPosition(0.5 + (botHeading/180));
    }
}