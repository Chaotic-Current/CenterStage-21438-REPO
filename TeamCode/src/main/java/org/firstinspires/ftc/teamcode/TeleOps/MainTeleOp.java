package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.*;


@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends LinearOpMode {
    //DO NOT MODIFY THIS CLASS WHATSOEVER UNLESS AUTHORIZED-> MAKE A COPY
    DriveTrain driveTrain;
    Outake outake;
    Intake intake;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    private void initialize() {

        driveTrain = new DriveTrain(hardwareMap,telemetry,gamepad1);
      //  outake = new Outake(hardwareMap,telemetry,gamepad2,driveTrain);
        intake  = new Intake(hardwareMap,telemetry,gamepad1,outake);


    }




    @Override
    public void runOpMode() {
        initialize();
        while (opModeInInit()) {
            driveTrain.IMUSetup();
        }
        while (opModeIsActive()) {

            intake.execute();
            outake.setIntake(intake.getCurrentState());
            outake.executeTeleOp();
            driveTrain.setIntake(intake.getCurrentState());
            driveTrain.drive();

        }

    }

}