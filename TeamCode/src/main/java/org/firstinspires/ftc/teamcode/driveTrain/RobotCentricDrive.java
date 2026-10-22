package org.firstinspires.ftc.teamcode.driveTrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Shared robot-centric power drive. The owning OpMode controls its lifecycle. */
public class RobotCentricDrive {

    private static final String FRONT_LEFT_NAME = "front_left_motor";
    private static final String FRONT_RIGHT_NAME = "front_right_motor";
    private static final String BACK_LEFT_NAME = "back_left_motor";
    private static final String BACK_RIGHT_NAME = "back_right_motor";

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

    public RobotCentricDrive(HardwareMap hardwareMap) {
        validateConstants();

        frontLeft = hardwareMap.get(DcMotorEx.class, FRONT_LEFT_NAME);
        frontRight = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT_NAME);
        backLeft = hardwareMap.get(DcMotorEx.class, BACK_LEFT_NAME);
        backRight = hardwareMap.get(DcMotorEx.class, BACK_RIGHT_NAME);

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
        telemetry.addLine("Robot centric: left stick moves relative to robot; right stick X turns.");
    }

    public void update(Gamepad gamepad) {
        double forward = applyDeadband(-gamepad.left_stick_y);
        double strafe = applyDeadband(gamepad.left_stick_x);
        double turn = applyDeadband(gamepad.right_stick_x);

        if (Math.abs(strafe) <= Math.abs(forward) * DIRECTION_SNAP_RATIO) {
            strafe = 0.0;
        } else if (Math.abs(forward) <= Math.abs(strafe) * DIRECTION_SNAP_RATIO) {
            forward = 0.0;
        }

        double frontLeftMix = forward + strafe + turn;
        double frontRightMix = forward - strafe - turn;
        double backLeftMix = forward - strafe + turn;
        double backRightMix = forward + strafe - turn;

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

    public void addTelemetry(Telemetry telemetry) {
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
