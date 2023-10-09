package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorTest extends OpenCvPipeline {





public Scalar sec1Scalar = new Scalar(0,0,0);




    @Override
    public Mat processFrame(Mat input) {

        Mat img = input;


        Rect sec1 = new Rect(50, 50, 200, 200);
        Point sec1top = new Point(50, 50);
        Point sec1bot = new Point(250, 250);


        Mat sec1MAT = new Mat(img, sec1);
        sec1Scalar = Core.mean(sec1MAT);
        Imgproc.rectangle(
                img,
                sec1top,
                sec1bot,
                sec1Scalar,
                6

        );


        // Release and return input

        return input;
    }

    // Returns an enum being the current position where the robot will park
    public String getColor() {
        if (sec1Scalar != null && sec1Scalar.val != null && sec1Scalar.val.length >= 3) {
            return (int) sec1Scalar.val[0] +" " + (int) sec1Scalar.val[1] + " "+  (int) sec1Scalar.val[2];
        }
        return "";
    }
}
