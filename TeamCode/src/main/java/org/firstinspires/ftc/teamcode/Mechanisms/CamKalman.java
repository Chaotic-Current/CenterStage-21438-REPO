package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.ejml.simple.SimpleMatrix;

public class CamKalman extends KalmanF{

    //variance of camera AprilTag data
    private double camXVar = 0.005;

    //variance of depth sensor
    private double depthYVar = 0.005;

    //velocity from odo
    private double velocityVar = 0.001;

    private Pose2d pastVelocity;

    //I think standard deivation of system
    private double sigma_a = 0.005;
    public CamKalman(Pose2d currentVel){
        F = new SimpleMatrix(new double[][]{
                {1, 0, dt, 0},
                {0, 1, 0, dt},
                {0, 0, 1, 0},
                {0, 0, 0, 1}
        });
        F_T = F.transpose();
        G = new SimpleMatrix(new double[][]{
                {0.5 * dt * dt},
                {0.5 * dt * dt},
                {dt},
                {dt}
        });
        R = new SimpleMatrix(new double[][]{
                {camXVar, 0, 0, 0},
                {0, depthYVar, 0, 0},
                {0, 0, velocityVar, 0},
                {0, 0, 0, velocityVar}
        });
        x_k_1 = new SimpleMatrix(new double[][]{
                {0},
                {0},
                {0},
                {0}
        });
        P_k_1 = new SimpleMatrix(new double[][]{
                {0.1, 0, 0, 0},
                {0, 0.1, 0, 0},
                {0, 0, 0.1, 0},
                {0, 0, 0, 0.1}
        });

        Q = G.mult(G.transpose()).scale(sigma_a * sigma_a);
        H = SimpleMatrix.identity(F.numRows());
        H_T = H.transpose();
        I = SimpleMatrix.identity(F.numRows());
        pastVelocity = new Pose2d(currentVel.getX(),currentVel.getY(), currentVel.getHeading());
    }

    public void inputUpdate(Pose2d currentVelocity, SimpleMatrix measurements){
        SimpleMatrix input  = new SimpleMatrix(new double[][]{
                {(1/dt)*(currentVelocity.getX()-pastVelocity.getX())},
                {(1/dt)*(currentVelocity.getY()-pastVelocity.getY())},
                {(1/dt)*(currentVelocity.getX()-pastVelocity.getX())},
                {(1/dt)*(currentVelocity.getY()-pastVelocity.getY())}
        });
        pastVelocity = new Pose2d(currentVelocity.getX(),currentVelocity.getY(),currentVelocity.getHeading());
        update(measurements,input);
    }

}
