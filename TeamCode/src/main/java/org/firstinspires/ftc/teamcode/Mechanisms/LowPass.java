package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;

@Config
public class LowPass {
    public static double alpha = 0.7;
    private double x;
    private double min = Integer.MAX_VALUE;
    private double max = Integer.MIN_VALUE;

    public LowPass(double u,double a){
        x = u;
        alpha = a;
    }

    public double execute(double u){
        x = x*alpha + (1-alpha)*u;
        return x;
    }

    public double range(){
        max  = x > max ? x : max;
        min = x < min ? x : min;

        return max - min;
    }

    public double getEstimate(){
        return x;
    }

    public void setEstimate(double n){
        x = n;
    }
}
