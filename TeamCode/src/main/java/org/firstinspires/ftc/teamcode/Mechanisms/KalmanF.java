package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.ejml.simple.SimpleMatrix;

import java.util.*;


@Config
public class KalmanF {
    //
    public SimpleMatrix x_k_1, P_k_1, K_k_1;
    //system paramters
    public SimpleMatrix F, G, R, Q, H, I, B, F_T, H_T;
    double dt = 0.025;//0.035


    private Pose2d pastPosition = new Pose2d(0,0,0);

    public static double depthVar = 0.001; // 0.005
    private double horzVar = 0.001; //0.005

    //velocity from odo
    public static double velocityVar = 0.0;



    //I think standard deivation of system
    private double sigma_a = 0.01;//0.005, 0.01

    private ArrayList pastVelo;

    public KalmanF(Pose2d currentVel){
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
                {horzVar, 0, 0, 0},
                {0, depthVar, 0, 0},
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
        B = new SimpleMatrix(new double[][]{
                {0.5*dt*dt,0,0,0},
                {0,0.5*dt*dt,0,0},
                {0,0,dt,0},
                {0,0,0,dt}
        });
        pastPosition = new Pose2d(currentVel.getX(),currentVel.getY(), currentVel.getHeading());
    }

    public KalmanF(SimpleMatrix f, SimpleMatrix g, SimpleMatrix r, SimpleMatrix q, SimpleMatrix h) {
        F = f;
        F_T = F.transpose();
        G = g;
        R = r;
        Q = q;
        H = h;
        H_T = H.transpose();
        I = SimpleMatrix.identity(F.numRows());
        pastVelo = new ArrayList<Double>(Arrays.asList(
                0.0,
                0.0
        ));
    }

    public void setInitalPostion(SimpleMatrix x_0, SimpleMatrix P_0, SimpleMatrix b) {
        x_k_1 = x_0;
        P_k_1 = P_0;
        setB(b);
    }

    public void setInitialPosition(SimpleMatrix x_0){
        x_k_1 = x_0;
    }

    public void setB(SimpleMatrix b){
         B = b;
    }


    /**
     * @param z - observation
     * @return
     */
    public SimpleMatrix update(SimpleMatrix z, SimpleMatrix u) {
        SimpleMatrix x = F.mult(x_k_1).plus((B.mult(u)));
        SimpleMatrix P = F.mult(P_k_1).mult(F_T).plus(Q);

        SimpleMatrix S = H.mult(P).mult(H_T).plus(R);
        SimpleMatrix K = P.mult(H_T).mult(S.invert());

        SimpleMatrix y = z.minus(H.mult(x));

        SimpleMatrix x_k = x.plus(K.mult(y));
        SimpleMatrix P_k = I.minus(K.mult(H)).mult(P);

        this.x_k_1 = x_k;
        this.P_k_1 = P_k;
        this.K_k_1 = K;


        return x_k;
    }
    public void inputUpdate( Pose2d currentPosition, SimpleMatrix measurements){
        /*
        SimpleMatrix input  = new SimpleMatrix(new double[][]{
                {(1/dt)*(currentPosition.getX()-pastVelocity.getX())},
                {(1/dt)*(currentPosition.getY()-pastVelocity.getY())},
                {(1/dt)*(currentPosition.getX()-pastVelocity.getX())},
                {(1/dt)*(currentPosition.getY()-pastVelocity.getY())}
        }); */
         /*
        SimpleMatrix input  = new SimpleMatrix(new double[][]{
                {currentPosition.getX()-pastPosition.getX())},
                {currentPosition.getY()-pastPosition.getY()},
                {0},
                {0}
        }); */
        SimpleMatrix input  = new SimpleMatrix(new double[][]{
                {0},
                {0},
                {0},
                {0}
        });

        pastPosition = new Pose2d(currentPosition.getX(),currentPosition.getY(),currentPosition.getHeading());
        update(measurements,input);
    }

    public SimpleMatrix getX_k(){
        return x_k_1;
    }
    public SimpleMatrix getK_k(){
        return K_k_1;
    }
}
