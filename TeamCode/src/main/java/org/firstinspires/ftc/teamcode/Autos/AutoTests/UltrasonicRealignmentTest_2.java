package org.firstinspires.ftc.teamcode.Autos.AutoTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
@Disabled
public class UltrasonicRealignmentTest_2 extends LinearOpMode {
    DistanceSensor dist;
    public double oldDistance;
    int count = 0;

    @Override
    public void runOpMode() {

        dist = hardwareMap.get(DistanceSensor.class, "Distance");
        oldDistance = dist.getDistance(DistanceUnit.INCH);
        ElapsedTime timer = new ElapsedTime();
        //double startTime;

        waitForStart();

        timer.reset();

        while (opModeIsActive() && !isStopRequested()) {

            if(dist.getDistance(DistanceUnit.INCH) < 2) {

                telemetry.addData("started", 1);
                //startTime=timer.seconds();
                telemetry.addData("time ", timer.seconds());
                //telemetry.addData("start time", startTime);
                 if(timer.seconds()  > 0.25){
                     telemetry.addData("yes", timer.seconds());
                     if(dist.getDistance(DistanceUnit.INCH) < 3 ){
                         count++;
                     }
                 }

            }
            telemetry.addData("distance in inches", dist.getDistance(DistanceUnit.INCH));
            telemetry.addData("count", count);
            telemetry.update();
        }
    }
}
