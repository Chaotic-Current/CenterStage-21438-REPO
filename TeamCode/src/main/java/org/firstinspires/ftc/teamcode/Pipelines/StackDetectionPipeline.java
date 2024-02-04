package org.firstinspires.ftc.teamcode.Pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class StackDetectionPipeline extends OpenCvPipeline{

    public static double blur = 5;

    private int sensativity = 50;

    public Scalar lowerBound = new Scalar(0, 0,255 -sensativity);

    public Scalar upperBound = new Scalar(180, sensativity, 255);

    private int width = 180;

    private double distance;

    private double xError;

    private Telemetry tel;

    public StackDetectionPipeline(Telemetry tel){
        this.tel = tel;
    }

    @Override
    public Mat processFrame(Mat input) {

        Mat mat = new Mat();

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);




        if (blur > 0 && blur % 2 == 1)
            Imgproc.GaussianBlur(mat, mat, new Size(blur, blur), 35);


        if (mat.empty()) {
            return input;
        }


        Mat thresh = new Mat();

        Core.inRange(mat, lowerBound, upperBound, thresh);

        Mat edges = new Mat();

        Imgproc.Canny(thresh, edges, 400, 1000);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_TC89_KCOS);

        double maxArea = -1;
        Rect[] boundRect = new Rect[contours.size()];
        int maxContourIdx = -1;
        for (int i = 0; i < contours.size(); i++) {
            double area = Imgproc.contourArea(contours.get(i));
            if (area > maxArea && area <900 && isCenterResonable(contours.get(i),input)) {
                maxArea = area;
                maxContourIdx = i;
            }
        }



        // Draw the largest contour on the original image
        if ((contours.size() >= 0) && maxContourIdx != -1) {
            Mat resultImage = input.clone();
            setDistance(contours, maxContourIdx);
            findError(contours, maxContourIdx,input);
            Imgproc.drawContours(resultImage, contours, maxContourIdx, new Scalar(255, 0, 0), 1);

            tel.addLine("" + distance);
            tel.update();

            return resultImage;
        }


        return input;
    }

    public void setDistance(List<MatOfPoint> contours, int i) {
        MatOfPoint x = contours.get(i);
        Rect y = Imgproc.boundingRect(x);
        tel.addLine("pixel width " + (y.width));

        distance = (3 * (980.0/3))/(y.width);// real focal length is 113.38582677
    }

    public void findError(List<MatOfPoint> contours, int i, Mat mat){
        MatOfPoint x = contours.get(i);
        Rect y = Imgproc.boundingRect(x);

        double pixelToInches = 3.0/(y.width);

        Moments moments = Imgproc.moments(x);

        double w = moments.m10 / moments.m00;

        xError = (w-mat.cols()/2.0)*pixelToInches;

        tel.addLine("Width " + mat.cols());
        tel.addLine("Center " + (mat.cols()/2.0));
        tel.addLine("pixel to inches " + pixelToInches);
        tel.addLine("x error : " + xError);

    }

    public boolean isCenterResonable(MatOfPoint countour, Mat mat){


        Moments moments = Imgproc.moments(countour);

        double y = moments.get_m01()/moments.get_m00();

        return y > mat.rows()/3*2;

    }
}
