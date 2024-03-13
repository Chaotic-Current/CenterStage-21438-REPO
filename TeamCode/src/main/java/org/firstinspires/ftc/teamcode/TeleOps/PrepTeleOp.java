package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Mechanisms.DriveTrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Outake;
import org.firstinspires.ftc.teamcode.Mechanisms.SlideMech;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;


@TeleOp(name = "PrepTeleOp")
@Config
public class PrepTeleOp extends LinearOpMode {

    /*
       SERVO POSITIONS:


       !!NOTE(when grabbing pixels and moving for deposit): LIFT ARM SLIGHTLY, THEN MOVE SLIDES, THEN FLIP ARM


       (armL and armR)DEPOSIT POSITION= 0.71
       (armL and armR)BEFORE PICKING POSITION= 0.1
       (armL and armR)PICKING POSITION= 0.


       (pivot) DEFAULT = 0.5

       (wrist) INTAKE = 0.09
       (wrist) DEPOSIT = 0.64

       (tilt) DEFAULT = 0.52

       (both claws) GRAB = 0.45
       (both claws) RELEASE = 0.0




   */
    //DO NOT MODIFY THIS CLASS WHATSOEVER UNLESS AUTHORIZED-> MAKE A COPY
    DriveTrain driveTrain;
    Outake outake;
    Intake intake;

    SlideMech slideMech;




    private Servo intakeArmR, intakeArmL, wrist, claw;



    private Servo armR, armL, top, bottom, gate;
    private PwmControl.PwmRange range;

    public static double intakeServoRPos = 0.5 + 0.06, intakeServoLPos = 0.82; //ignore the 0.06 it's for jank

    public static double intakePos = 0.5;

    public static double armRPos = 0.9, armLPos = 0.9, gatePos = 0.5;

    public static int targetPos = 1000;


    //0.94 for deposit pose, 0.12 for intake pose


    public static double wristPos = 0.5;

    public static double clawPosT = 0.5, clawPosB = 0.5;

    //0.6 half open, 0.7 close


    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    private void initialize() {




       /* armL = hardwareMap.get(Servo.class, "armL");
        armR = hardwareMap.get(Servo.class, "armR");
        armL.setDirection(Servo.Direction.REVERSE); */

       // outake = new Outake(hardwareMap,telemetry,gamepad2,driveTrain);
       // intake = new Intake(hardwareMap,telemetry,gamepad1,outake);
        gate = hardwareMap.servo.get("flicker");



    }




    @Override
    public void runOpMode() {
        initialize();
        while (opModeInInit()) {
           // driveTrain.IMUSetup();
        }
        while (opModeIsActive()) {
            //intakeArmL.setPosition(intakeServoLPos);
            gate.setPosition(gatePos);
            //outake.executeTeleOp();
            //intake.executeTeleOp();
           //.825 half open (openClawB)
            //1 closeClawU & clawB
            //0.7 open ClawU5
            //
          // telemetry.addData("pos:",slideMech.getCurrentTickPosition());

            //claw.setPosition(clawPos);
          // intake.executeTeleOp();
           // intake.execute();
           // outake.setIntake(intake.getCurrentState());
           // outake.executeTeleOp();
           // wrist.setPosition(wristPos);
            //armR.setPosition(armRPos);
            //armL.setPosition(armLPos);

           // intakeArmR.setPosition(intakeServoRPos);
           // intakeArmL.setPosition(intakeServoLPos);
            //armR.setPosition(armRPos);
            //armL.setPosition(armLPos);
            //wrist.setPosition(wristPos);
            //claw.setPosition(clawPos);
           // slideMech.update(telemetry);
            //intake.execute();
            //outake.setIntake(intake.getCurrentState());
            //outake.executeTeleOp();
            //driveTrain.setIntake(intake.getCurrentState());
            //driveTrain.drive();
            telemetry.update();

        }

    }

}