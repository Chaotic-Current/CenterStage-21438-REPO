package org.firstinspires.ftc.teamcode.Mechanisms;

public class LowPass {
    private double alpha = 0.7;
    private double x;

    public LowPass(double u,double a){
        x = u;
        alpha = a;
    }

    public double execute(double u){
        x = x*alpha + (1-alpha)*u;
        return x;
    }
    public double getEstimate(){
        return x;
    }

    public void setEstimate(double n){
        x = n;
    }
}
