package org.firstinspires.ftc.teamcode.Mechanisms;

import org.ejml.simple.SimpleMatrix;

public class KalmanF {
    //
    public SimpleMatrix x_k_1, P_k_1;
    //system paramters
    public SimpleMatrix F, G, R, Q, H, I, B, F_T, H_T;
    double dt = 0.025;

    public KalmanF(){}

    public KalmanF(SimpleMatrix f, SimpleMatrix g, SimpleMatrix r, SimpleMatrix q, SimpleMatrix h) {
        F = f;
        F_T = F.transpose();
        G = g;
        R = r;
        Q = q;
        H = h;
        H_T = H.transpose();
        I = SimpleMatrix.identity(F.numRows());

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



        return x_k;
    }
}
