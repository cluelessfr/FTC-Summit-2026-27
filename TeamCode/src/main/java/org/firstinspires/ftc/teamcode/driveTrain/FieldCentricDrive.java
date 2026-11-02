package org.firstinspires.ftc.teamcode.driveTrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.localization.InitializePinpoint;

public class FieldCentricDrive {

    private static final String FRONT_LEFT_NAME = "front_left_motor";
    private static final String FRONT_RIGHT_NAME = "front_right_motor";
    private static final String BACK_LEFT_NAME = "back_left_motor";
    private static final String BACK_RIGHT_NAME = "back_right_motor";
    private static final double JOYSTICK_DEADBAND = 0.05;
    private static final double DIRECTION_SNAP_DEGREES = 20.0;
    private static final double DIRECTION_SNAP_RATIO =
            Math.tan(Math.toRadians(DIRECTION_SNAP_DEGREES));

    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    private final InitializePinpoint pinpoint;
    private double headingOffsetRadians = 0;
    private boolean resetYawWasPressed;
    private double holdHeadingRadians;
    private boolean headingHoldActive = false;
    private static final double HEADING_HOLD_KP = 0.5;
    private static final double MAX_HOLD_TURN = 0.2;

    public FieldCentricDrive(HardwareMap hardwareMap, InitializePinpoint pinpoint) {
        validateConstants();

        frontLeft = hardwareMap.get(DcMotorEx.class, FRONT_LEFT_NAME);
        frontRight = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT_NAME);
        backLeft = hardwareMap.get(DcMotorEx.class, BACK_LEFT_NAME);
        backRight = hardwareMap.get(DcMotorEx.class, BACK_RIGHT_NAME);
        this.pinpoint = pinpoint;

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        configureForPowerControl(frontLeft);
        configureForPowerControl(frontRight);
        configureForPowerControl(backLeft);
        configureForPowerControl(backRight);
    }

    public void addInitializationTelemetry(Telemetry telemetry) {
        telemetry.addData("Drive mode", "Field centric with Pinpoint");
        telemetry.addData("Pod offsets (mm)", "X %.2f / Y %.2f",
                pinpoint.getXPodOffset(), pinpoint.getYPodOffset());
        telemetry.addLine("Drive heading uses Pinpoint with a driver heading offset.");
        telemetry.addLine("Gamepad 1: left stick moves; right stick X turns.");
        telemetry.addLine("Gamepad 1 D-pad up: reset drive heading; stop before resetting.");
        telemetry.addData("Drive direction snap (deg)", "%.1f", DIRECTION_SNAP_DEGREES);
        addTelemetry(telemetry);
    }

    public void update(Gamepad gamepad) {
        boolean resetYawPressed = gamepad.dpad_up;
        if (resetYawPressed && !resetYawWasPressed) {
            resetHeading();
        }
        resetYawWasPressed = resetYawPressed;

        double fieldForward = applyDeadband(-gamepad.left_stick_y);
        double fieldStrafe = applyDeadband(gamepad.left_stick_x);
        double turn = applyDeadband(gamepad.right_stick_x);

        double currentHeading = pinpoint.getHeadingRadians();
        boolean driverTurning = turn != 0.0;
        boolean driverMoving = fieldForward != 0.0 || fieldStrafe != 0.0;

        if (driverTurning || !driverMoving) {
            headingHoldActive = false;
        }
        else {
            if (!headingHoldActive) {
                holdHeadingRadians = currentHeading;
                headingHoldActive = true;
            }

            double error = holdHeadingRadians - currentHeading;

            // Choose the shortest rotation, including across the ±180° boundary.
            error = Math.atan2(Math.sin(error), Math.cos(error));

            turn = Math.max(-MAX_HOLD_TURN, Math.min(MAX_HOLD_TURN, HEADING_HOLD_KP * error));
        }

        // Snap in field coordinates before rotating into the robot's heading.
        if (Math.abs(fieldStrafe) <= Math.abs(fieldForward) * DIRECTION_SNAP_RATIO) {
            fieldStrafe = 0.0;
        } else if (Math.abs(fieldForward) <= Math.abs(fieldStrafe) * DIRECTION_SNAP_RATIO) {
            fieldForward = 0.0;
        }

        double headingRadians =
                pinpoint.getHeadingRadians() - headingOffsetRadians;

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
        headingOffsetRadians = pinpoint.getHeadingRadians();
        headingHoldActive = false;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("Gamepad 1 D-pad up: reset drive heading");
        telemetry.addData("Drive heading (deg)", "%.1f",
                Math.toDegrees(pinpoint.getHeadingRadians() - headingOffsetRadians));
        telemetry.addData("Drive heading offset (deg)", "%.1f", Math.toDegrees(headingOffsetRadians));
        telemetry.addData("FL / FR commanded power (%)", "%.0f / %.0f",
                frontLeft.getPower() * 100.0, frontRight.getPower() * 100.0);
        telemetry.addData("BL / BR commanded power (%)", "%.0f / %.0f",
                backLeft.getPower() * 100.0, backRight.getPower() * 100.0);
        telemetry.addLine();
        pinpoint.addTelemetry(telemetry);
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
