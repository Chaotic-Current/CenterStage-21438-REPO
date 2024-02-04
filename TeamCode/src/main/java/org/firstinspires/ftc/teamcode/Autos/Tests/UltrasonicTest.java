package org.firstinspires.ftc.teamcode.Autos.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp
public class UltrasonicTest extends OpMode {
    private AnalogDistanceDriver rightUltra, leftUltra; // ctrl+click on AnalogDistanceDriver for class explanation
    private double leftDist; // in
  //private double rightDist; // in
    private double angle;
    private final double DIST_BTWN_SENS = 10; // in

    @Override
    public void init() {
        leftUltra = new AnalogDistanceDriver(hardwareMap.get(AnalogInput.class, "leftUltra"),telemetry);
        //rightUltra = new AnalogDistanceDriver(hardwareMap.get(AnalogInput.class, "rightUltra"));
    }

    @Override
    public void loop(){
        leftDist = leftUltra.getDistance();
        // rightDist = rightUltra.getDistance();
        // angle = Math.atan(Math.abs(leftDist-rightDist) / DIST_BTWN_SENS);

        telemetry.addData("left distance (in): ", leftDist);
        //telemetry.addData("right distance (in): ", rightDist);
        telemetry.addData("angle (degrees): ", Math.toDegrees(angle)); // atan returns in radians
        telemetry.update();
    }
}