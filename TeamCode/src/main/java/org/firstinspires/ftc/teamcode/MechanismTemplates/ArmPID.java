package org.firstinspires.ftc.teamcode.MechanismTemplates;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ArmPID {

    private PIDFController armPIDF;
    private CRServo arm;
    private HardwareMap hardwareMap;
    public static double armKpUp = 0.0000001; // old is 0.012
    public static double armKpDown = 0.000001;
    public static double armKi = 0;
    public static double armKd = 0;
    public static double armKf = 0;
    public static double EXTAKE_POS = 120; // 180 old val; in degrees of absolute encoder
    public static double INTAKE_POS = 64; // 65 old val
    public static double targetPos = 255;
    private double lastError = 0;


    public ArmPID(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        arm = new CRServo(hardwareMap,"axon");
        armPIDF = new PIDFController(armKpUp, armKi, armKd, armKf);
    }

    public void update(Telemetry telemetry, ElapsedTime timer) {
        /*
        double correction = armPIDF.calculate(getArmPosition(), targetPos);
       telemetry.addData("Correction: ", correction);
        telemetry.addData("Target Position: ", targetPos);
        telemetry.addData("Motor Position: ", getArmPosition());
        telemetry.update();
        arm.set(correction); // sets a PID-tuned voltage for the arm motor
        */
        double error = Math.abs(getArmPosition() - targetPos);

        double derivitve = (error - lastError)/timer.seconds();

        if(getArmPosition() < targetPos){
            arm.set(error * 0.2 + derivitve * armKd) ;
        } else if (getArmPosition() > targetPos){
            arm.set(error * -0.2 + derivitve * armKd);
        }

        lastError = error;

        /*
        double correction = 0;

        if(!(Math.abs(getArmPosition() - targetPos) <= 10)){
            double error = targetPos - getArmPosition();

            double derivative = (error - lastError)/timer.seconds();

            integralSum += (error * timer.seconds());

           correction = (armKpUp * error) ;

            arm.set(correction);

            lastError = error;
        }
        */


        telemetry.addData("Correction: ", 0);
        telemetry.addData("Target Position: ", targetPos);
        telemetry.addData("Motor Position: ", getArmPosition());
        telemetry.update();
    }


    private double getArmPosition() {
        if(hardwareMap.analogInput.get("axonSensor").getVoltage() < 0.6){
            return Math.abs(hardwareMap.analogInput.get("axonSensor").getVoltage() - 0.6)/3.3 * 360;
        } else{
            return (Math.abs(hardwareMap.analogInput.get("axonSensor").getVoltage() - 3.3)/3.3 * 360 + (0.6/3.3 * 360));
        }
        //return (int)((hardwareMap.analogInput.get("axonSensor").getVoltage()/3.3) * 360);
    }

    public void setExtake() {
        armPIDF.setPIDF(armKpUp, armKi, armKd, armKf);
        targetPos = EXTAKE_POS;
    }
    public void setIntake() {
        armPIDF.setPIDF(armKpDown, armKi, armKd, armKf);
        targetPos = INTAKE_POS;
    }

    public void setCustom(int custom) {
        armPIDF.setPIDF(armKpUp, armKi, armKd, armKf);
        targetPos = custom;
    }


}