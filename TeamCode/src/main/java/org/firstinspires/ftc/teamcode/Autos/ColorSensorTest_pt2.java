package org.firstinspires.ftc.teamcode.Autos;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Pipelines.DetectRed;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.MechanismTemplates.ColorSensorMech;

@Autonomous
@Config
public class ColorSensorTest_pt2 extends LinearOpMode{
    ColorSensorMech color1;
    ColorSensorMech color2;
    double oldRed;
    double oldBlue;
    double oldGreen;
    int count = 0;

    @Override
    public void runOpMode() {


        color1 = new ColorSensorMech(hardwareMap, "Color");
        //color2 = new ColorSensorMech(hardwareMap, "Color");
        //color.enableLed(false);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if(color1.redDeviation(oldRed) || color1.blueDeviation(oldBlue) || color1.greenDeviation(oldGreen)){
                count++;
            }
            oldRed = color1.getRed();
            oldBlue= color1.getBlue();
            oldGreen= color1.getGreen();

            telemetry.addData(" c1 Red", color1.getRed());
            telemetry.addData(" c1 Green", color1.getGreen());
            telemetry.addData(" c1 Blue", color1.getBlue());
            telemetry.addData("count", count);

            //color1.update(telemetry);
            telemetry.update();
        }

    }
}