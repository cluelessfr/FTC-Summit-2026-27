package org.firstinspires.ftc.teamcode.driveTrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private double headingErrorRadians;
    private double turnCorrection;
    private double translationScale = 1;
    private static final double HEADING_HOLD_KP = 1;
    private static final double MAX_HOLD_TURN = 0.2;
    private static final double HEADING_SETTLE_TOLERANCE = Math.toRadians(1);
    private static final double HEADING_SETTLE_SECONDS = 0.25;
    private static final double HEADING_STOP_TIMEOUT_SECONDS = 2.0;
    private final ElapsedTime headingStopTimer = new ElapsedTime();
    private final ElapsedTime headingSettleTimer = new ElapsedTime();

    private static final double TRANSLATION_KP = 0.01; // power per inch/sec of sideways drift
    private static final double MAX_TRANSLATION_CORRECTION = 0.15;
    private double sidewaysDriftSpeed;
    private double measuredForwardSpeed;
    private double measuredStrafeSpeed;
    private double forwardCorrection;
    private double strafeCorrection;
    private boolean translationCorrectionActive;

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
        telemetry.addLine("GP1: left stick move; right stick turn; D-pad up reset heading.");
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
        headingErrorRadians = 0;
        turnCorrection = 0;

        if (driverMoving || driverTurning) {
            headingStopTimer.reset();
            headingSettleTimer.reset();
        }

        if (driverTurning) {
            headingHoldActive = false;
        }
        else if (driverMoving || headingHoldActive) {
            // Keep holding after releasing the stick to correct drift while stopping.
            if (!headingHoldActive) {
                holdHeadingRadians = currentHeading;
                headingHoldActive = true;
            }

            double error = holdHeadingRadians - currentHeading;

            // Choose the shortest rotation, including across the ±180° boundary.
            error = Math.atan2(Math.sin(error), Math.cos(error));

            headingErrorRadians = error;
            if (Math.abs(error) > HEADING_SETTLE_TOLERANCE) {
                headingSettleTimer.reset();
            }
            // Release after settling, or after the timeout, until the driver moves again.
            if (!driverMoving && (headingStopTimer.seconds() >= HEADING_STOP_TIMEOUT_SECONDS
                    || headingSettleTimer.seconds() >= HEADING_SETTLE_SECONDS)) {
                headingHoldActive = false;
                headingErrorRadians = 0;
            } else {
                turnCorrection = clip(-HEADING_HOLD_KP * error, MAX_HOLD_TURN);
            }
            turn = turnCorrection;
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

        // Pinpoint uses field X/Y with left positive; drive strafe is right positive.
        double velocityX = pinpoint.getXVelocityInches();
        double velocityY = pinpoint.getYVelocityInches();
        double fieldCosine = Math.cos(currentHeading);
        double fieldSine = Math.sin(currentHeading);
        measuredForwardSpeed = velocityX * fieldCosine + velocityY * fieldSine;
        measuredStrafeSpeed = velocityX * fieldSine - velocityY * fieldCosine;

        forwardCorrection = 0;
        strafeCorrection = 0;
        sidewaysDriftSpeed = 0;
        double movementMagnitude = Math.hypot(robotForward, robotStrafe);
        translationCorrectionActive = movementMagnitude > 0 && pinpoint.isReady()
                && Double.isFinite(measuredForwardSpeed) && Double.isFinite(measuredStrafeSpeed);
        if (translationCorrectionActive) {
            // Correct only motion perpendicular to the stick direction, leaving speed to the driver.
            double perpendicularForward = -robotStrafe / movementMagnitude;
            double perpendicularStrafe = robotForward / movementMagnitude;
            sidewaysDriftSpeed = measuredForwardSpeed * perpendicularForward
                    + measuredStrafeSpeed * perpendicularStrafe;
            double correction = clip(-TRANSLATION_KP * sidewaysDriftSpeed, MAX_TRANSLATION_CORRECTION);
            forwardCorrection = correction * perpendicularForward;
            strafeCorrection = correction * perpendicularStrafe;
            robotForward += forwardCorrection;
            robotStrafe += strafeCorrection;
        }

        // Leave room for heading correction while moving instead of scaling the turn down.
        translationScale = 1;
        double translationMagnitude = Math.abs(robotForward) + Math.abs(robotStrafe);
        if (headingHoldActive && translationMagnitude > 0) {
            translationScale = Math.min(1.0, (1.0 - Math.abs(turn)) / translationMagnitude);
            robotForward *= translationScale;
            robotStrafe *= translationScale;
        }

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
        headingErrorRadians = 0;
        turnCorrection = 0;
    }

    public void addTelemetry(Telemetry telemetry) {
        pinpoint.addTelemetry(telemetry);
        telemetry.addData("Drive heading (deg)", "%.1f",
                Math.toDegrees(pinpoint.getHeadingRadians() - headingOffsetRadians));
        telemetry.addData("Heading hold", headingHoldActive ? "ON; error %.1f deg" : "OFF",
                Math.toDegrees(headingErrorRadians));
        telemetry.addData("Drift correction", translationCorrectionActive ? "ON" : "OFF");
    }

    private void configureForPowerControl(DcMotorEx motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < JOYSTICK_DEADBAND ? 0.0 : value;
    }

    private double clip(double value, double limit) {
        return Math.max(-limit, Math.min(limit, value));
    }

    public void stop() {
        headingHoldActive = false;
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
