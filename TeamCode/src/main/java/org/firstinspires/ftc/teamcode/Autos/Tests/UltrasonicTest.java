package org.firstinspires.ftc.teamcode.Autos.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp
public class UltrasonicTest extends OpMode {
    private AnalogDistanceDriver rightUltra, leftUltra; // ctrl+click on AnalogDistanceDriver for class explanation
    private double leftDist; // cm
    private double rightDist; // cm
    private double angle;
    private final double DIST_BTWN_SENS = 24; // cm

    @Override
    public void init() {
        leftUltra = new AnalogDistanceDriver(hardwareMap.get(AnalogInput.class, "leftUltra"));
        rightUltra = new AnalogDistanceDriver(hardwareMap.get(AnalogInput.class, "rightUltra"));
    }

    @Override
    public void loop(){
        leftDist = leftUltra.getDistance(); // returns distance in cm
        rightDist = rightUltra.getDistance();

        angle = Math.atan(Math.abs(leftDist-rightDist) / DIST_BTWN_SENS);

        telemetry.addData("left distance (cm): ", leftDist);
        telemetry.addData("right distance (cm): ", rightDist);
        telemetry.addData("angle (degrees): ", Math.toDegrees(angle)); // atan returns in radians
        telemetry.update();
    }
}