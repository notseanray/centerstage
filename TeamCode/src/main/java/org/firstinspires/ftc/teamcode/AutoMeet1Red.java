package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
 * driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * The code assumes a Robot Configuration with motors named: leftfront_drive and rightfront_drive, leftback_drive and rightback_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID (usually starting at 0)
 *
 * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
 * Manually drive the robot until it displays Target data on the Driver Station.
 *
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */

@Autonomous
public class AutoMeet1Red extends LinearOpMode {


    private String webcamName = "Webcam 1";
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 14.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.4;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.4;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.2;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor LeftFront   = null;  //  Used to control the left front drive wheel
    private DcMotor RightFront  = null;  //  Used to control the right front drive wheel
    private DcMotor LeftBack    = null;  //  Used to control the left back drive wheel
    private DcMotor RightBack   = null;  //  Used to control the right back drive wheel

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static int DESIRED_TAG_ID = 0;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    public boolean targetFound     = false;
    public double  drive           = 0;        // Desired forward power/speed (-1 to +1)
    private OpenCvCamera camera;
    public double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
    public double  turn            = 0;
    public static double RightFEC = 0;
    public static double LeftFEC = 0;
    public static double RightBEC = 0;
    public static double LeftBEC = 0;
    public static double LiftET = 0;


    // Guess and Check this value
    static final double COUNTS_PER_ROT = 384.5;
    // 12"/circumfrence of wheel * COUNTS_PER_ROT
    static final double COUNTS_PER_TILE = 12.0 / (Math.PI * (96.0 / 25.4)) * COUNTS_PER_ROT;
    static final int COUNTS_PER_LIFT_INCH = 1;
    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        LeftFront  = hardwareMap.get(DcMotor.class, "motor_ch_3");
        RightFront = hardwareMap.get(DcMotor.class, "motor_eh_0");
        LeftBack  = hardwareMap.get(DcMotor.class, "motor_ch_2");
        RightBack = hardwareMap.get(DcMotor.class, "motor_ch_1");

//        DcMotor liftLeft = hardwareMap.dcMotor.get("motor_eh_3");
//        DcMotor liftRight = hardwareMap.dcMotor.get("motor_eh_2");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightBack.setDirection(DcMotor.Direction.FORWARD);
        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        ColorDetection colorDetection;

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu2");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);



        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();


        System.out.println("camera");
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);

        System.out.println("color");
        colorDetection = new ColorDetection();
        System.out.println("color pipeline");
        camera.setPipeline(colorDetection);



        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640  ,360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        ColorDetection.ParkingPosition position = colorDetection.getPosition();
        System.out.println("parking");

        while (!isStarted()) {
            position = colorDetection.getPosition();
            telemetry.addData("Cube Position: ", String.valueOf(position));
            telemetry.update();
        }
        while (position == ColorDetection.ParkingPosition.NONE) {
            position = colorDetection.getPosition();
        }

        camera.closeCameraDevice();
        waitForStart();

        imu.resetYaw();
        if (position == ColorDetection.ParkingPosition.LEFT) {
            DESIRED_TAG_ID = 4;
            System.out.println("1");
            EncoderFB(0.5, 0.5, 0.5, 0.5);
//                        EncoderTurn(30);
            IMUTurn(35, imu);
            EncoderFB(1.4, 1.4, 1.4, 1.4);
            EncoderFB(-0.5, -0.5, -0.5, -0.5);
            //   Lift(2);
            //   Lift(-2);
            IMUTurn(-80, imu);
        } else if (colorDetection.getPosition() == ColorDetection.ParkingPosition.CENTER) {
            DESIRED_TAG_ID = 5;
            EncoderFB(2.5, 2.5, 2.5, 2.5);
            EncoderFB(-0.3, -0.3, -0.3, -0.3);
            IMUTurn(-80, imu);
            //  Lift(2);
            //drop
            //  Lift(-2);
        } else {
            DESIRED_TAG_ID = 6;
            EncoderFB(0.5, 0.5, 0.5, 0.5);
            IMUTurn(-30, imu);
            EncoderFB(1, 1, 1, 1);
            EncoderFB(-0.5, -0.5, -0.5, -0.5);
            //  Lift(2);
            //drop
            //  Lift(-2);
            IMUTurn(-80, imu);
        }
        EncoderFB(1, 1, 1, 1);
        telemetry.addData("Initializing AprilTag Pipeline in Camera", 1);
        initAprilTag();
        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        detectTag(imu);
        IMUTurn(-80, imu);
        EncoderFB(-1, 1, 1, -1);
        EncoderFB(1, 1, 1, 1);



//        EncoderFB(2, 2, 2, 2);
//
//        // Lift(6);
//        //drop
//        // Lift(-6);
//        IMUTurn(90, imu);
//        EncoderFB(1,1,1,1);
//        IMUTurn(-90, imu);
//        EncoderFB(1,1,1,1);
        while (opModeIsActive()) {
            telemetry.addData("Auto Done", 1);
            telemetry.update();
        }
    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void detectTag(IMU imu) {
        targetFound = false;
        desiredTag = null;
        boolean initalized = false;


        while (opModeIsActive()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null) &&
                        ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID))) {
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
                }
            }
            if (targetFound) {
                telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);

                initalized = true;

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;
                if (Math.abs(rangeError) < 5 && Math.abs(headingError) < 10 && Math.abs(yawError) < 10) {
                    break;
                }
                // prevent weirdness, aka FIRST code is SHIT
                if (Math.abs(Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) - 90) > 30) {
                    break;
                }

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                telemetry.update();
                moveRobot(drive, strafe, turn);
                sleep(10);
            } else {
                LeftFront.setPower(0);
                RightFront.setPower(0);
                LeftBack.setPower(0);
                RightBack.setPower(0);
                if (initalized) {
                    break;
                }
            }
        }
        LeftFront.setPower(0);
        RightFront.setPower(0);
        LeftBack.setPower(0);
        RightBack.setPower(0);
    }
        public void moveRobot(double x, double y, double yaw) {
            // Calculate wheel powers.
//            double leftFrontPower    =  x +y +yaw;
//            double leftBackPower     =  y -x +yaw;
//            double rightFrontPower   =  y -x -yaw;
//            double rightBackPower    =  y +x -yaw;
            double leftFrontPower    =  x -y -yaw;
            double leftBackPower     =  x +y -yaw;
            double rightFrontPower   =  x +y +yaw;
            double rightBackPower    =  x -y +yaw;

            // Normalize wheel powers to be less than 1.0
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send powers to the wheels.
            LeftFront.setPower(leftFrontPower);
            RightFront.setPower(rightFrontPower);
            LeftBack.setPower(leftBackPower);
            RightBack.setPower(rightBackPower);
        }

        /**
         * Initialize the AprilTag processor.
         */
        private void initAprilTag() {
            // Create the AprilTag processor by using a builder.
            aprilTag = new AprilTagProcessor.Builder().build();

            // Create the vision portal by using a builder.
                visionPortal = new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                        .addProcessor(aprilTag)
                        .build();
            }


    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
        private void  setManualExposure(int exposureMS, int gain) {
            // Wait for the camera to be open, then use the controls

            if (visionPortal == null) {
                return;
            }

            // Make sure camera is streaming before we try to set the exposure controls
            if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                telemetry.addData("Camera", "Waiting");
                telemetry.update();
                while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                    sleep(20);
                }
                telemetry.addData("Camera", "Ready");
                telemetry.update();
            }

            // Set camera controls unless we are stopping.
            if (!isStopRequested())
            {
                ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                    sleep(50);
                }
                exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
                sleep(20);
                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                gainControl.setGain(gain);
                sleep(20);
            }
        }
    public void EncoderFB(double LF, double LB, double RF, double RB) {
        RightFEC = RF * COUNTS_PER_TILE;
        RightBEC = RB * COUNTS_PER_TILE;
        LeftFEC = LF * COUNTS_PER_TILE;
        LeftBEC = LB * COUNTS_PER_TILE;

        RightBack.setTargetPosition((int) RightBEC + RightBack.getCurrentPosition());
        RightFront.setTargetPosition((int) RightFEC + RightFront.getCurrentPosition());
        LeftBack.setTargetPosition((int) LeftBEC + LeftBack.getCurrentPosition());
        LeftFront.setTargetPosition((int) LeftFEC + LeftFront.getCurrentPosition());

        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftFront.setPower(0.3);
        LeftBack.setPower(0.3);
        RightFront.setPower(0.3);
        RightBack.setPower(0.3);

        while (RightBack.isBusy() || RightFront.isBusy() || LeftBack.isBusy() || LeftFront.isBusy()) {
            telemetry.addData("Running to position", 1);
            telemetry.update();
        }
        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);

        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void IMUTurn(double angle, IMU imu) {
        double targetAngle = angle;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        while (Math.abs(Math.abs(botHeading) - Math.abs(targetAngle)) > 2.0) {
            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double rx = Math.min(Math.max((((targetAngle - botHeading) / 180.0)), .2), 0.4);
            telemetry.addData("Rotating, angle: ", botHeading);
            telemetry.addData("Rotating, target: ", targetAngle);
            telemetry.update();
//            double rx = Math.max(Math.min((((Math.abs(targetAngleRadians) - Math.abs(botHeading)) / 180.0)), .125), 0.3);
            if (targetAngle < 0.0) {
                rx *= -1.0;
            }
            LeftFront.setPower(-rx);
            LeftBack.setPower(-rx);
            RightFront.setPower(rx);
            RightBack.setPower(rx);
        }
        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);
    }

    public void EncoderTurn(double degrees) {
        if (degrees > 0) {
            //turn left
            RightFEC += degrees * COUNTS_PER_ROT;
            RightBEC += degrees * COUNTS_PER_ROT;
            LeftFEC -= degrees * COUNTS_PER_ROT;
            LeftBEC -= degrees * COUNTS_PER_ROT;


        } else {
            RightFEC -= degrees * COUNTS_PER_ROT;
            RightBEC -= degrees * COUNTS_PER_ROT;
            LeftFEC += degrees * COUNTS_PER_ROT;
            LeftBEC += degrees * COUNTS_PER_ROT;


        }
        RightBack.setTargetPosition((int) RightBEC);
        RightFront.setTargetPosition((int) RightFEC);
        LeftBack.setTargetPosition((int) LeftBEC);
        LeftFront.setTargetPosition((int) LeftFEC);

        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftFront.setPower(0.3);
        LeftBack.setPower(0.3);
        RightFront.setPower(0.3);
        RightBack.setPower(0.3);

        while (RightBack.isBusy() && RightFront.isBusy() && LeftBack.isBusy() && LeftFront.isBusy()) {
            telemetry.addData("Running to position", 1);
            telemetry.update();

        }

        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);

        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}