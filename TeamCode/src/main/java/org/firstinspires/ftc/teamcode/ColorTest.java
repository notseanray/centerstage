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
        int leftX = 50;
        int leftY = 180;
        int leftWidth = 100;
        int leftHeight = 90;

        int centerX = 300;
        int centerY = 180;
        int centerWidth = 100;
        int centerHeight = 90;

        int rightX = 540;
        int rightY = 180;
        int rightWidth = 100;
        int rightHeight = 90;

        Rect left = new Rect(leftX , leftY, leftWidth, leftHeight);
        Rect center = new Rect(centerX, centerY, centerWidth, centerHeight);
        Rect right = new Rect(rightX, rightY, rightWidth, rightHeight);

        Mat leftMAT = new Mat(img, left);
        Mat centerMAT = new Mat(img, center);
        Mat rightMAT= new Mat(img, right);



        Scalar leftScalar = Core.mean(leftMAT);
        Scalar centerScalar = Core.mean(centerMAT);
        Scalar rightScalar = Core.mean(rightMAT);


        Point leftTop = new Point(
                leftX,
                leftY
        );
        Point leftBottom = new Point(
                leftX + leftWidth,
                leftY + leftHeight
        );
        Point centerTop = new Point(
                centerX,
                centerY
        );
        Point centerBottom = new Point(
                centerX + centerWidth,
                centerY + centerHeight
        );
        Point rightTop = new Point(
                rightX,
                rightY
        );
        Point rightBottom = new Point(
                rightX + rightWidth,
                rightY + rightHeight
        );

        Imgproc.rectangle(
                img,
                rightTop,
                rightBottom,
                new Scalar(rightScalar.val[0], rightScalar.val[1], rightScalar.val[2]),
                10

        );
        Imgproc.rectangle(
                img,
                leftTop,
                leftBottom,
                new Scalar(leftScalar.val[0], leftScalar.val[1], leftScalar.val[2]),
                2

        );
        Imgproc.rectangle(
                img,
                centerTop,
                centerBottom,
                new Scalar(centerScalar.val[0], centerScalar.val[1], centerScalar.val[2]),
                2

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
