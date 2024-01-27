package org.firstinspires.ftc.teamcode.MechanismTemplates.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MechanismTemplates.IntakeMech;

@Autonomous
public class AutoIntakeLogicTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        IntakeMech intake = new IntakeMech(hardwareMap);
       ElapsedTime timer = new ElapsedTime();
       ElapsedTime delay = new ElapsedTime();

        waitForStart();
        timer.reset();
        delay.reset();

        int counter = 0;
        intake.start();

            while(timer.seconds()<6)
            {

                if(intake.getIntakeVoltage()>0.5 && delay.seconds() > 0.2){
                    counter++;
                    delay.reset();
                }
                if(counter>2){
                    intake.reverse();
                }

                telemetry.addData("voltage", intake.getIntakeVoltage());

            }
    }
}
