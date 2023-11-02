package org.firstinspires.ftc.teamcode;
import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.Util.setupIMU;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Mecanum", group="Drive Systems")
public class MecanumDrive extends LinearOpMode {
    final static double STICK_DEADZONE = 0.08;
    double angleOffset = 0;
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
        CRServo intakeRight = hardwareMap.crservo.get("servo_5_ch");
        CRServo intakeLeft = hardwareMap.crservo.get("servo_5_eh");
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu2");
        byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis

//Need to be in CONFIG mode to write to registers
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);

        sleep(100); //Changing modes requires a delay before doing anything else

//Write to the AXIS_MAP_CONFIG register
        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE & 0x0F);

//Write to the AXIS_MAP_SIGN register
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0x0F);

//Need to change back into the IMU mode to use the gyro
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);

        sleep(100); //Changing modes again requires a delay
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Adjust the orientation parameters to match your robot
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
            if (Math.abs(y) < STICK_DEADZONE) {
                y = 0;
            }
            if (Math.abs(x) < STICK_DEADZONE) {
                x = 0;
            }
            if (Math.abs(rx) < STICK_DEADZONE) {
                rx = 0;
            }
            double botHeading = imu.getAngularOrientation().firstAngle - angleOffset;
            // TODO power curve for input

//            System.out.println(y);
            if (gamepad1.y) {
                angleOffset = botHeading;
            }
            // TODO airplane
            if (gamepad1.x) {
               plane.setPosition(0.8);
            } else {
                plane.setPosition(0);
            }
            if (gamepad1.right_bumper) {
                intakeRight.setPower(0.8);
                intakeLeft.setPower(0.8);
            }

            if (gamepad1.left_bumper) {
                intakeRight.setPower(-0.8);
                intakeLeft.setPower(-0.8);
            }
            if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
                intakeRight.setPower(0);
                intakeLeft.setPower(0);
            }


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
