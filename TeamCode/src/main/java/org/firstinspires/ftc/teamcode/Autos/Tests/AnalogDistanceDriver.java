package org.firstinspires.ftc.teamcode.Autos.Tests;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AnalogDistanceDriver {

    private AnalogInput analog;
    private Telemetry telemetry;
    //private int maxR = 500;  Stands for "max range", which is the max measurement value of the Gravity ultrasonic in centimeters
    // old 520

    public AnalogDistanceDriver(AnalogInput analog, Telemetry tel) {
        this.analog = analog;
        telemetry = tel;
    }

    public double getDistance() {
        // FOR MaxBotix ULTRASONIC
        double distance = analog.getVoltage() * 87.5 - 10.875;
        telemetry.addData("Voltage ",analog.getVoltage());

        // (ignore) FOR GRAVITY ULTRASONIC:
        // The product wiki lists the distance calculation as [Vout(mV)×520/Vin(mV)]
        // in this case, Vout is the analog reading (that's just how analog sensor work), and Vin (votlage in)
        // is 3.3 since that's the pintout of the actual sensor

        return distance;
    }

    //[messed up equation] distance = [vobserved / ((Vcc/1024) * 6)] - 300
}