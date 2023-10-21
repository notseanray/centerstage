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
        RIGHT,
        NONE
    }




    private ParkingPosition position = ParkingPosition.NONE;

    @Override
    public Mat processFrame(Mat input) {

        Mat img = input;
        int leftX = 2;
        int leftY = 180;
        int leftWidth = 100;
        int leftHeight = 90;

        int centerX = 300;
        int centerY = 180;
        int centerWidth = 60;
        int centerHeight = 60;

        int rightX = 548;
        int rightY = 180;
        int rightWidth = 90;
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


        Scalar ColorToMatch = new Scalar(255, 0, 0);
//        double leftChannelDistance = Math.abs(ColorToMatch.val[0] - leftScalar.val[0]) + ColorToMatch.val[1] - leftScalar.val[1] + ColorToMatch.val[2] - leftScalar.val[2];
//        double centerChannelDistance = ColorToMatch.val[0] - centerScalar.val[0] + ColorToMatch.val[1] - centerScalar.val[1] + ColorToMatch.val[2] - centerScalar.val[2];
//        double rightChannelDistance = ColorToMatch.val[0] - rightScalar.val[0] + ColorToMatch.val[1] - rightScalar.val[1] + ColorToMatch.val[2] - rightScalar.val[2];
        double maxLeft = Math.min(leftScalar.val[0], Math.min(leftScalar.val[1], leftScalar.val[2]));
        double maxCenter = Math.min(centerScalar.val[0], Math.min(centerScalar.val[1], centerScalar.val[2]));
        double maxRight = Math.min(rightScalar.val[0], Math.min(rightScalar.val[1], rightScalar.val[2]));
        // normalize to unit vector, then dotproduct
        double leftChannelDistance = ColorToMatch.val[0] * leftScalar.val[0] / maxLeft + ColorToMatch.val[1] * leftScalar.val[1] / maxLeft + ColorToMatch.val[2] * leftScalar.val[2] / maxLeft;
        double centerChannelDistance = ColorToMatch.val[0] * centerScalar.val[0] / maxCenter + ColorToMatch.val[1] * centerScalar.val[1] / maxCenter + ColorToMatch.val[2] * centerScalar.val[2] / maxCenter;
        double rightChannelDistance = ColorToMatch.val[0] * rightScalar.val[0] / maxRight + ColorToMatch.val[1] * rightScalar.val[1] / maxRight + ColorToMatch.val[2] * rightScalar.val[2] / maxRight;



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

        double maxDistance = Math.max(leftChannelDistance, Math.max(centerChannelDistance, rightChannelDistance));
        if (leftChannelDistance == maxDistance) {
            position = ParkingPosition.LEFT;
            Imgproc.rectangle(
                    img,
                    leftTop,
                    leftBottom,
                    new Scalar(leftScalar.val[0], leftScalar.val[1], leftScalar.val[2]),
                    10

            );
            Imgproc.rectangle(
                    img,
                    centerTop,
                    centerBottom,
                    new Scalar(centerScalar.val[0], centerScalar.val[1], centerScalar.val[2]),
                    2

            );
            Imgproc.rectangle(
                    img,
                    rightTop,
                    rightBottom,
                    new Scalar(rightScalar.val[0], rightScalar.val[1], rightScalar.val[2]),
                    2

            );

        } else if (centerChannelDistance == maxDistance) {
            position = ParkingPosition.CENTER;
            System.out.println("2");
            Imgproc.rectangle(
                    img,
                    centerTop,
                    centerBottom,
                    new Scalar(centerScalar.val[0], centerScalar.val[1], centerScalar.val[2]),
                    10

            );
            Imgproc.rectangle(
                    img,
                    rightTop,
                    rightBottom,
                    new Scalar(rightScalar.val[0], rightScalar.val[1], rightScalar.val[2]),
                    2

            );
            Imgproc.rectangle(
                    img,
                    leftTop,
                    rightBottom,
                    new Scalar(leftScalar.val[0], leftScalar.val[1], leftScalar.val[2]),
                    2

            );

        } else if (rightChannelDistance == maxDistance) {
            position = ParkingPosition.RIGHT;
            System.out.println("3");
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
        }

        // Release and return input

        return input;
    }

    // Returns an enum being the current position where the robot will park
    public ParkingPosition getPosition() {
        return position;
    }
}
