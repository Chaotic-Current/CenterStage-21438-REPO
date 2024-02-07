package org.firstinspires.ftc.teamcode.Mechanisms.MechanismTests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Mechanisms.AnalogDistanceDriver;
import org.firstinspires.ftc.teamcode.Mechanisms.LowPass;

@TeleOp
public class UltrasonicTest extends OpMode {
    private AnalogDistanceDriver rightUltra, leftUltra; // ctrl+click on AnalogDistanceDriver for class explanation
    private double leftDist; // -> inches
    private double filtered;
    private LowPass lowPass;
  //private double rightDist;
    private double angle;
    private final double DIST_BTWN_SENS = 10; // -> in

    @Override
    public void init() {
        leftUltra = new AnalogDistanceDriver(hardwareMap.get(AnalogInput.class, "leftUltra"),telemetry);
        lowPass = new LowPass(0,0.975);
        //rightUltra = new AnalogDistanceDriver(hardwareMap.get(AnalogInput.class, "rightUltra"));
    }

    @Override
    public void loop(){
        leftDist = leftUltra.getDistance();
        filtered = lowPass.execute(leftUltra.getDistance());
        // rightDist = rightUltra.getDistance();
        // angle = Math.atan(Math.abs(leftDist-rightDist) / DIST_BTWN_SENS);

        telemetry.addData("left distance (in): ", leftDist);
        telemetry.addData("Filtered distance ", filtered);
        telemetry.addData("Range ", lowPass.range());
        //telemetry.addData("right distance (in): ", rightDist);
        telemetry.addData("angle (degrees): ", Math.toDegrees(angle)); // -> radians
        telemetry.update();
    }
}