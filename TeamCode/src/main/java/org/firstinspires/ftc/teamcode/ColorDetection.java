package org.firstinspires.ftc.teamcode;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorDetection extends OpenCvPipeline {



    public enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT
    }




    private volatile ParkingPosition position = ParkingPosition.LEFT;

    @Override
    public Mat processFrame(Mat input) {

        Mat img = input;

        Rect sec1 = new Rect(200 / 2, 180 /1, 200 / 2, 180 / 2);
        Rect sec2 = new Rect(600 / 2, 180 / 1, 200 / 2, 180 / 2);
        Rect sec3 = new Rect(1000 / 2, 180 / 1, 200 / 2, 180 / 2);

        Mat sec1MAT = new Mat(img, sec1);
        Mat sec2MAT = new Mat(img, sec2);
        Mat sec3MAT = new Mat(img, sec3);



        Scalar sec1Scalar = Core.mean(sec1MAT);
        Scalar sec2Scalar = Core.mean(sec2MAT);
        Scalar sec3Scalar = Core.mean(sec3MAT);


        Scalar ColorToMatch = new Scalar(0, 0, 255);
        double Sec1AvgDist = Math.abs(ColorToMatch.val[0] - sec1Scalar.val[0]) + Math.abs(ColorToMatch.val[1] - sec1Scalar.val[1]) + Math.abs(ColorToMatch.val[2] - sec1Scalar.val[2]);
        double Sec2AvgDist = (Math.abs(ColorToMatch.val[0] - sec2Scalar.val[0]) + Math.abs(ColorToMatch.val[1] - sec2Scalar.val[1]) + Math.abs(ColorToMatch.val[2] - sec2Scalar.val[2]));
        double Sec3AvgDist = (Math.abs(ColorToMatch.val[0] - sec3Scalar.val[0]) + Math.abs(ColorToMatch.val[1] - sec3Scalar.val[1]) + Math.abs(ColorToMatch.val[2] - sec3Scalar.val[2]));


        Point sec1top = new Point(
                200 / 2,
                180 / 1
        );
        Point sec1bot = new Point(
                400 / 2,
                360 / 1
        );
        Point sec2top = new Point(
                600 / 2,
                180 / 1
        );
        Point sec2bot = new Point(
                800 / 2,
                360 / 1
        );
        Point sec3top = new Point(
                1000 / 2,
                180 / 1
        );
        Point sec3bot = new Point(
                1200 / 2,
                360 / 1
        );


        if (Sec1AvgDist < Sec2AvgDist && Sec1AvgDist < Sec3AvgDist) {
            System.out.println("1");
            position = ParkingPosition.LEFT;
            Imgproc.rectangle(
                    img,
                    sec1top,
                    sec1bot,
                    new Scalar(sec1Scalar.val[0], sec1Scalar.val[1], sec1Scalar.val[2]),
                    10

            );
            Imgproc.rectangle(
                    img,
                    sec2top,
                    sec2bot,
                    new Scalar(sec2Scalar.val[0], sec2Scalar.val[1], sec2Scalar.val[2]),
                    2

            );
            Imgproc.rectangle(
                    img,
                    sec3top,
                    sec3bot,
                    new Scalar(sec3Scalar.val[0], sec3Scalar.val[1], sec3Scalar.val[2]),
                    2

            );

        } else if (Sec2AvgDist < Sec3AvgDist) {
            position = ParkingPosition.CENTER;
            System.out.println("2");
            Imgproc.rectangle(
                    img,
                    sec2top,
                    sec2bot,
                    new Scalar(sec2Scalar.val[0], sec2Scalar.val[1], sec2Scalar.val[2]),
                    10

            );
            Imgproc.rectangle(
                    img,
                    sec3top,
                    sec3bot,
                    new Scalar(sec3Scalar.val[0], sec3Scalar.val[1], sec3Scalar.val[2]),
                    2

            );
            Imgproc.rectangle(
                    img,
                    sec1top,
                    sec1bot,
                    new Scalar(sec1Scalar.val[0], sec1Scalar.val[1], sec1Scalar.val[2]),
                    2

            );

        } else {
            position = ParkingPosition.RIGHT;
            System.out.println("3");
            Imgproc.rectangle(
                    img,
                    sec3top,
                    sec3bot,
                    new Scalar(sec3Scalar.val[0], sec3Scalar.val[1], sec3Scalar.val[2]),
                    10

            );
            Imgproc.rectangle(
                    img,
                    sec1top,
                    sec1bot,
                    new Scalar(sec1Scalar.val[0], sec1Scalar.val[1], sec1Scalar.val[2]),
                    2

            );
            Imgproc.rectangle(
                    img,
                    sec2top,
                    sec2bot,
                    new Scalar(sec2Scalar.val[0], sec2Scalar.val[1], sec2Scalar.val[2]),
                    2

            );
        }

        // Release and return input

        return input;
    }

    // Returns an enum being the current position where the robot will park
    public ParkingPosition getPosition() {
        return position;
    }
}
