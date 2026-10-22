package org.firstinspires.ftc.teamcode.driveTrain;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class FieldCentricDrive {

    private static final String FRONT_LEFT_NAME = "front_left_motor";
    private static final String FRONT_RIGHT_NAME = "front_right_motor";
    private static final String BACK_LEFT_NAME = "back_left_motor";
    private static final String BACK_RIGHT_NAME = "back_right_motor";
    private static final String IMU_NAME = "imu";

    // Change these two values to match the Control Hub's actual mounting on the robot.
    private static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private static final RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

    private static final double JOYSTICK_DEADBAND = 0.05;
    // Snap translation within 20 degrees of forward, backward, left, or right.
    // Increase for more straight-line assistance; decrease for easier off-axis movement.
    private static final double DIRECTION_SNAP_DEGREES = 20.0;
    private static final double DIRECTION_SNAP_RATIO =
            Math.tan(Math.toRadians(DIRECTION_SNAP_DEGREES));

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private IMU imu;

    private boolean resetYawWasPressed;

    public FieldCentricDrive(HardwareMap hardwareMap) {
        validateConstants();

        frontLeft = hardwareMap.get(DcMotorEx.class, FRONT_LEFT_NAME);
        frontRight = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT_NAME);
        backLeft = hardwareMap.get(DcMotorEx.class, BACK_LEFT_NAME);
        backRight = hardwareMap.get(DcMotorEx.class, BACK_RIGHT_NAME);
        imu = hardwareMap.get(IMU.class, IMU_NAME);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        configureForPowerControl(frontLeft);
        configureForPowerControl(frontRight);
        configureForPowerControl(backLeft);
        configureForPowerControl(backRight);

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                LOGO_FACING_DIRECTION, USB_FACING_DIRECTION);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
    }

    public void addInitializationTelemetry(Telemetry telemetry) {
        telemetry.addData("Drive mode", "Field centric; left stick moves, right stick X turns");
        telemetry.addData("Hub orientation", "Logo %s, USB %s",
                LOGO_FACING_DIRECTION, USB_FACING_DIRECTION);
        telemetry.addLine("Hub IMU heading is relative to the last yaw reset.");
        telemetry.addLine("D-pad down: reset field heading; stop before resetting.");
        telemetry.update();
    }

    public void update(Gamepad gamepad) {
        boolean resetYawPressed = gamepad.dpad_down;
        if (resetYawPressed && !resetYawWasPressed) {
            imu.resetYaw();
        }
        resetYawWasPressed = resetYawPressed;

        double fieldForward = applyDeadband(-gamepad.left_stick_y);
        double fieldStrafe = applyDeadband(gamepad.left_stick_x);
        double turn = applyDeadband(gamepad.right_stick_x);

        // Snap in field coordinates before rotating into the robot's heading.
        if (Math.abs(fieldStrafe) <= Math.abs(fieldForward) * DIRECTION_SNAP_RATIO) {
            fieldStrafe = 0.0;
        } else if (Math.abs(fieldForward) <= Math.abs(fieldStrafe) * DIRECTION_SNAP_RATIO) {
            fieldForward = 0.0;
        }

        double headingRadians =
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the field-relative translation command by negative robot heading.
        double cosine = Math.cos(headingRadians);
        double sine = Math.sin(headingRadians);
        double robotForward = fieldForward * cosine - fieldStrafe * sine;
        double robotStrafe = fieldStrafe * cosine + fieldForward * sine;

        double frontLeftMix = robotForward + robotStrafe + turn;
        double frontRightMix = robotForward - robotStrafe - turn;
        double backLeftMix = robotForward - robotStrafe + turn;
        double backRightMix = robotForward + robotStrafe - turn;

        // Scale every wheel together so the requested movement direction is preserved.
        double largestMagnitude = Math.max(1.0,
                Math.max(
                        Math.max(Math.abs(frontLeftMix), Math.abs(frontRightMix)),
                        Math.max(Math.abs(backLeftMix), Math.abs(backRightMix))));

        frontLeft.setPower(frontLeftMix / largestMagnitude);
        frontRight.setPower(frontRightMix / largestMagnitude);
        backLeft.setPower(backLeftMix / largestMagnitude);
        backRight.setPower(backRightMix / largestMagnitude);
    }

    public void resetHeading() {
        imu.resetYaw();
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Drive heading (Hub IMU)", "%.1f deg",
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addLine("D-pad down: reset field heading");
        telemetry.addData("FL / FR commanded power", "%.2f / %.2f",
                frontLeft.getPower(), frontRight.getPower());
        telemetry.addData("BL / BR commanded power", "%.2f / %.2f",
                backLeft.getPower(), backRight.getPower());
    }

    private void configureForPowerControl(DcMotorEx motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < JOYSTICK_DEADBAND ? 0.0 : value;
    }

    public void stop() {
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    private void validateConstants() {
        if (!Double.isFinite(DIRECTION_SNAP_DEGREES)
                || DIRECTION_SNAP_DEGREES < 0.0 || DIRECTION_SNAP_DEGREES >= 45.0) {
            throw new IllegalArgumentException("DIRECTION_SNAP_DEGREES must be in [0, 45).");
        }
        if (JOYSTICK_DEADBAND < 0.0 || JOYSTICK_DEADBAND >= 1.0) {
            throw new IllegalArgumentException("JOYSTICK_DEADBAND must be in [0, 1).");
        }
    }
}
