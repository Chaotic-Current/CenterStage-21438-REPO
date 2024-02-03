package org.firstinspires.ftc.teamcode.Autos.Tests;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class AnalogDistanceDriver {

    private AnalogInput analog;
    private int maxR = 520; // Stands for "max range", which is the max measurement value of the ultrasonic in centimeters

    public AnalogDistanceDriver(AnalogInput analog) {
        this.analog = analog;
    }

    public double getDistance() {
        double distance = (this.analog.getVoltage()*maxR)/3.3;
        // The product wiki lists the distance calculation as [Vout(mV)×520/Vin(mV)]
        // in this case, Vout is the analog reading (that's just how analog sensor work), and Vin (votlage in)
        // is 3.3 since that's the pintout of the actual sensor

        return distance * 2.54; // convert cm to inches
    }
}