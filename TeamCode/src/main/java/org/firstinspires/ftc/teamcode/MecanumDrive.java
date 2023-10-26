package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Mecanum", group="Drive Systems")
public class MecanumDrive extends LinearOpMode {
    final static double STICK_DEADZONE = 0.08;
    @Override
    public void runOpMode() throws InterruptedException {
        Servo plane = hardwareMap.get(Servo.class, "plane");
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("motor_ch_3");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("motor_ch_2");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("motor_eh_0");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("motor_ch_1");

        DcMotor liftLeft = hardwareMap.dcMotor.get("motor_eh_3");
        DcMotor liftRight = hardwareMap.dcMotor.get("motor_eh_2");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu2");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            liftLeft.setPower(gamepad1.left_trigger);
            liftRight.setPower(gamepad1.right_trigger);
            // TODO deadzone variable
            double left_trigger = gamepad1.left_trigger;
            double right_trigger = gamepad1.right_trigger;
            if (left_trigger > 0.05) {
                liftRight.setPower(-left_trigger);
                liftLeft.setPower(left_trigger);
            } else if (right_trigger > 0.05) {
                liftRight.setPower(right_trigger);
                liftLeft.setPower(-right_trigger);
            } else {
                liftLeft.setPower(0);
                liftRight.setPower(0);
            }
            double y = -gamepad1.right_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.right_stick_x;
            double rx = gamepad1.left_stick_x;
            if (y < STICK_DEADZONE) {
                y = 0;
            }
            if (x < STICK_DEADZONE) {
                x = 0;
            }
            if (rx < STICK_DEADZONE) {
                rx = 0;
            }
            // TODO power curve for input

//            System.out.println(y);
            if (gamepad1.y) {
                imu.resetYaw();
            }
            // TODO airplane
            if (gamepad1.x) {
               plane.setPosition(0.8);
            } else {
                plane.setPosition(0);
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;
            telemetry.addData("Rotating, angle: ", botHeading * (180.0 / Math.PI));
            telemetry.update();

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}
