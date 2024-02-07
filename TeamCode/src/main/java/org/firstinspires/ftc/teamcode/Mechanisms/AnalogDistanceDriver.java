package org.firstinspires.ftc.teamcode.Mechanisms;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AnalogDistanceDriver {

    private AnalogInput analog;
    private Telemetry telemetry;
    //private int maxR = 520;  Stands for "max range", which is the max measurement value of the Gravity ultrasonic in centimeters

    public AnalogDistanceDriver(AnalogInput analog, Telemetry tel) {
        this.analog = analog;
        telemetry = tel;
    }

    public double getDistance() {
        double distance = analog.getVoltage() * 87.5 - 10.875;
        telemetry.addData("Voltage ",analog.getVoltage());

        // (ignore)
        // GRAVITY ULTRASONIC:
        // Distance Calculation: [Vout(mV)×520/Vin(mV)]

        return distance; // -> inches
    }

    //[sus equation] distance = [vobserved / ((Vcc/1024) * 6)] - 300
}