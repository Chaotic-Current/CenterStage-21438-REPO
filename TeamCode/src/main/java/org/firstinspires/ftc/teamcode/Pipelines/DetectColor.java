package org.firstinspires.ftc.teamcode.Pipelines;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class DetectColor extends OpenCvPipeline {
    public enum ColorLocation {
        UNDETECTED, //Constants on the location of red in the camera frame, not seen, left in frame, right in frame, center in frame
        LEFT,
        RIGHT,
        CENTER
    }

    private double avgColorWidth;
    private Scalar upperBound, lowerBound;
    private final int width;//width of the image
    public int blur = 13;
    private boolean isBlue;
    private Telemetry telemetry;
    private ColorLocation locate;

    //no real use ngl, just to make EOCV-Sim work properly

    public DetectColor(int w, Telemetry tel) {
        width = w;
        telemetry = tel;

    }

    public DetectColor(int width, Telemetry telemetry, Scalar upper, Scalar lower){
         this.width = width;
         this.telemetry = telemetry;
         upperBound = upper;
         lowerBound = lower;
         //locate = ColorLocation.UNDETECTED;
    }

    public DetectColor(int width, Telemetry telemetry, Scalar upper, Scalar lower, boolean isBlue){
        this.width = width;
        this.telemetry = telemetry;
        upperBound = upper;
        lowerBound = lower;
        this.isBlue = isBlue;
        //locate = ColorLocation.UNDETECTED;
    }

    public DetectColor(Telemetry tel) { //never gonna use this constructor in the real world
        isBlue = true;
        telemetry = tel;
        width = 180;
        lowerBound = new Scalar(75, 100, 100);
        upperBound = new Scalar(140, 255, 255);
    }

    @Override
    public Mat processFrame(Mat input) { // all color detection will be happening here
        /**Mat means matrix, each matrix is essentially a frame from the camera that will be used as the image to detect color in
         * this pipeline, this will return a matrix that can be seen on the DS camera stream, the matrix returned comes from the camera
         */

        Mat mat = new Mat();//copy of the input matrix
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);//changing the color space from RGB to HSV, due to it being more useful when light values change

        if (mat.empty()) { //if mat is shown not to have anything, as a failsafe the location would be undetected and will return the input matrix
            locate = ColorLocation.UNDETECTED;
            return input;
        }

        if (blur > 0 && blur % 2 == 1)
            Imgproc.GaussianBlur(mat, mat, new Size(blur, blur), 35);

        // The HSV values for the lower bound of red, and the higher bound of red
        //Values from https://cvexplained.wordpress.com/2020/04/28/color-detection-hsv/#:~:text=The%20HSV%20values%20for%20true,10%20and%20160%20to%20180.
//       Scalar lowerBound = new Scalar(2, 105, 105);
//        Scalar upperBound = new Scalar(10, 255, 255);

        Mat thresh = new Mat();

        // We'll get a black and white image. The white regions represent red.
        // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
        Core.inRange(mat, lowerBound, upperBound, thresh);


        Mat masked = new Mat();
        //color the white portion of thresh in with HSV from mat
        //output into masked
        Core.bitwise_and(mat, mat, masked, thresh);
        //calculate average HSV values of the white thresh values
        Scalar average = Core.mean(masked, thresh);

        Mat scaledMask = new Mat();
        //scale the average saturation to 150
        masked.convertTo(scaledMask, -1, 150 / average.val[1], 0);

        /**
         * Everything up till this point was to isolate all red entities from all non red entities,
         * now we need to decide if the red entities is in the center of the frame,left, right, or not shown
         */

        //Using the Canny edge detector, we can detect the edges of the tape to make a boundary box later
        //Its just the red tape that should show up, so we don't need to be too strict with the threshold
        Mat edges = new Mat();
        Imgproc.Canny(thresh, edges, 500, 1000);// https://docs.opencv.org/3.4/da/d5c/tutorial_canny_detector.html for anybody wondering how this method works


        //While the edges should not be disconnected in this case (just a line of tape), could be potential to find disconected edges
        //Use the findContours method to reconnect the edges
        //Use the contours to find the bounding rectangles
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        //Creating the bounding rectangle
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];


        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }

        /**
         * The edges of the tape has been created, now all that needs to happen is to use the width of the
         * image to find the relative location of the tape to the image (i.e. towards the left, right, or center)
         */


        double leftImage = 0.2 * width;//the left of the image can be classified as everything bellow this value
        double rightImage = 0.6 * width;//the right of the image can be classified as everything above this value

        if(isBlue){
            leftImage = 0.4 * width;
            rightImage = 0.8 * width;
        }
        //TODO need to tune these values to make sure they actually work and done under or overshoot

        boolean left = false;//conditionals for the if statements later
        boolean right = false;
        boolean center = false;


        //A for loop to determine where the tape is using the contour array length
       /* for (int i = 0; i != boundRect.length; i++) {
            if (boundRect[i].x < leftImage)
                left = true;
            if (boundRect[i].x + boundRect[i].width > rightImage)
                right = true;

        }*/

        double maxArea = -1;
        int maxContourIdx = -1;
        for (int i = 0; i < contours.size(); i++) {
            double area = Imgproc.contourArea(contours.get(i));
            if (area > maxArea  && area > 0) {
                maxArea = area;
                maxContourIdx = i;
            }
        }

        if(maxContourIdx == -1){
            locate = ColorLocation.UNDETECTED;

            return scaledMask;
        }

        Point centerPoint = findContourCenter(contours, maxContourIdx);

        if(centerPoint != null) {

            if (centerPoint.x <= leftImage)
                left = true;
            else if (centerPoint.x >= rightImage)
                right = true;

            //checking if the object is in the center if it is not on the left, nor right of the image and if the color we want to detect exists in the image
            center = !left && !right && (contours.size() > 0);


            //officially stating what the location of the tape is
            if (left) {
                locate = ColorLocation.LEFT;
            } else if (right ) {
                locate = ColorLocation.RIGHT;
            } else if (center && avgColorWidth > (isBlue ? 95 : 35)) {
                locate = ColorLocation.CENTER;
            } else {
                locate = ColorLocation.UNDETECTED;
            }
            telemetry.update();

            telemetry.addData("Location pre run", locate);
            telemetry.addData("Center location: ", avgColorWidth);
            telemetry.update();


            if (!contours.isEmpty()) {
                MatOfPoint contour = null;
                //if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.N) {
                contour = Collections.max(contours, Comparator.comparingDouble(Imgproc::contourArea));
                // }
                Moments moments = Imgproc.moments(contour);
                avgColorWidth = moments.get_m10() / moments.get_m00();
            }

            if (!contours.isEmpty()) {
                Mat resultImage = input.clone();
                Imgproc.drawContours(resultImage, contours, maxContourIdx, new Scalar(255, 0, 0), 1);




                return resultImage;
            }
        }

        locate = ColorLocation.UNDETECTED;

        return scaledMask;  //displaying edges of all red objects cuz i think it looks cool
    }

    //A method so that other classes can get the relative location of the tape
    public ColorLocation getLocate() {
        return locate;
    }

    public double getDistanceFromMidPoint() {
        return (width/2.0)-avgColorWidth;
    }

    public Point findContourCenter(List<MatOfPoint> contours, int i) {
        if (!contours.isEmpty()) {
            MatOfPoint select = contours.get(i);

            Moments moments = Imgproc.moments(select);

            double x = moments.m10 / moments.m00;
            double y = moments.m01 / moments.m00;

            return new Point(x, y);
        }

        return null;
    }
}