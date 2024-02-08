package org.firstinspires.ftc.teamcode.Mechanisms.MechanismTests;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Mechanisms.LowPass;

@TeleOp(name = "REV2mTest")
public class REV2mTest  extends OpMode {
    Rev2mDistanceSensor left, right;

    private double leftDist, leftFiltered;
    private double rightDist, rightFiltered;

    private LowPass lowPass;
    private double angle;
    private final double DIST_BTWN_SENS = 10;

    @Override
    public void init() {
        left = hardwareMap.get(Rev2mDistanceSensor.class, "left");
        right = hardwareMap.get(Rev2mDistanceSensor.class, "right");

        lowPass = new LowPass(0,0.975); // TODO: tune values
    }

    @Override
    public void loop(){
        leftDist = left.getDistance(DistanceUnit.INCH);
        rightDist = right.getDistance(DistanceUnit.INCH);

        leftFiltered = lowPass.execute(leftDist);
        rightFiltered = lowPass.execute(rightDist);

        telemetry.addData("filtered left dist:", leftFiltered);
        telemetry.addData("raw left dist:", leftDist);

        telemetry.addData("filtered right dist:", rightFiltered);
        telemetry.addData("raw right dist:", rightDist);

        telemetry.addData("filtered angle:", Math.atan(Math.abs(leftFiltered-rightFiltered) / DIST_BTWN_SENS));

        telemetry.update();
    }
}

