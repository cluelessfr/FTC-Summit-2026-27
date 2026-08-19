package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Field-centric mecanum TeleOp that commands encoder velocity instead of raw motor power.
 *
 * Before running:
 * 1. Select the correct motor type for every drive motor in the Robot Configuration.
 * 2. Connect all four motor encoder cables.
 * 3. Match the hardware names below to the Robot Configuration.
 * 4. Enter the actual front chain sprocket tooth counts below.
 * 5. Set the Control Hub logo and USB directions to match its physical mounting.
 * 6. Test motor directions with the wheels raised and at a low MAX_SPEED_FRACTION.
 */
@TeleOp(name = "Field-Centric Mecanum (Velocity)", group = "Drive")
public class FieldCentricMecanumVelocity extends LinearOpMode {

    private static final String FRONT_LEFT_NAME = "front_left_drive";
    private static final String FRONT_RIGHT_NAME = "front_right_drive";
    private static final String BACK_LEFT_NAME = "back_left_drive";
    private static final String BACK_RIGHT_NAME = "back_right_drive";
    private static final String IMU_NAME = "imu";

    /*
     * For a chain drive:
     * wheel revolutions per motor revolution = motor sprocket teeth / wheel sprocket teeth.
     * Leaving both values equal represents a 1:1 chain ratio.
     */
    private static final double FRONT_MOTOR_SPROCKET_TEETH = 1.0;
    private static final double FRONT_WHEEL_SPROCKET_TEETH = 1.0;

    // The rear wheels are directly driven, so one motor revolution is one wheel revolution.
    private static final double REAR_WHEEL_REVS_PER_MOTOR_REV = 1.0;

    // Change these two values to match the Control Hub's actual mounting on the robot.
    private static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private static final RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // Begin conservatively. Increase only after checking target versus measured RPM under load.
    private static final double MAX_SPEED_FRACTION = 0.60;
    private static final double JOYSTICK_DEADBAND = 0.05;

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private IMU imu;

    private double frontWheelRevsPerMotorRev;
    private double maximumWheelRpm;
    private boolean resetYawWasPressed;

    @Override
    public void runOpMode() {
        validateConstants();

        frontLeft = hardwareMap.get(DcMotorEx.class, FRONT_LEFT_NAME);
        frontRight = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT_NAME);
        backLeft = hardwareMap.get(DcMotorEx.class, BACK_LEFT_NAME);
        backRight = hardwareMap.get(DcMotorEx.class, BACK_RIGHT_NAME);
        imu = hardwareMap.get(IMU.class, IMU_NAME);

        /*
         * These directions match the FTC mecanum sample. If a wheel moves backward during the
         * raised-wheel forward test, reverse only that wheel's direction here.
         */
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        configureForVelocityControl(frontLeft);
        configureForVelocityControl(frontRight);
        configureForVelocityControl(backLeft);
        configureForVelocityControl(backRight);

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                LOGO_FACING_DIRECTION, USB_FACING_DIRECTION);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        frontWheelRevsPerMotorRev =
                FRONT_MOTOR_SPROCKET_TEETH / FRONT_WHEEL_SPROCKET_TEETH;

        // Use the slowest wheel's achievable speed so all four wheels can reach every command.
        maximumWheelRpm = MAX_SPEED_FRACTION * minimumAchievableWheelRpm();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Maximum wheel RPM", "%.1f", maximumWheelRpm);
        telemetry.addData("Front chain ratio", "%.3f wheel rev / motor rev",
                frontWheelRevsPerMotorRev);
        telemetry.addData("Hub orientation", "Logo %s, USB %s",
                LOGO_FACING_DIRECTION, USB_FACING_DIRECTION);
        telemetry.addLine("The robot's starting direction becomes field forward.");
        telemetry.addLine("Press A while stopped to redefine field forward.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        // Define zero heading from the direction in which the robot starts TeleOp.
        imu.resetYaw();

        while (opModeIsActive()) {
            boolean resetYawPressed = gamepad1.a;
            if (resetYawPressed && !resetYawWasPressed) {
                imu.resetYaw();
            }
            resetYawWasPressed = resetYawPressed;

            double fieldForward = applyDeadband(-gamepad1.left_stick_y);
            double fieldStrafe = applyDeadband(gamepad1.left_stick_x);
            double turn = applyDeadband(gamepad1.right_stick_x);

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

            double frontLeftWheelRpm =
                    maximumWheelRpm * frontLeftMix / largestMagnitude;
            double frontRightWheelRpm =
                    maximumWheelRpm * frontRightMix / largestMagnitude;
            double backLeftWheelRpm =
                    maximumWheelRpm * backLeftMix / largestMagnitude;
            double backRightWheelRpm =
                    maximumWheelRpm * backRightMix / largestMagnitude;

            setWheelRpm(frontLeft, frontLeftWheelRpm, frontWheelRevsPerMotorRev);
            setWheelRpm(frontRight, frontRightWheelRpm, frontWheelRevsPerMotorRev);
            setWheelRpm(backLeft, backLeftWheelRpm, REAR_WHEEL_REVS_PER_MOTOR_REV);
            setWheelRpm(backRight, backRightWheelRpm, REAR_WHEEL_REVS_PER_MOTOR_REV);

            telemetry.addData("Heading", "%7.1f deg", Math.toDegrees(headingRadians));
            telemetry.addData("Target FL / FR wheel RPM", "%7.1f / %7.1f",
                    frontLeftWheelRpm, frontRightWheelRpm);
            telemetry.addData("Actual FL / FR wheel RPM", "%7.1f / %7.1f",
                    getWheelRpm(frontLeft, frontWheelRevsPerMotorRev),
                    getWheelRpm(frontRight, frontWheelRevsPerMotorRev));
            telemetry.addData("Target BL / BR wheel RPM", "%7.1f / %7.1f",
                    backLeftWheelRpm, backRightWheelRpm);
            telemetry.addData("Actual BL / BR wheel RPM", "%7.1f / %7.1f",
                    getWheelRpm(backLeft, REAR_WHEEL_REVS_PER_MOTOR_REV),
                    getWheelRpm(backRight, REAR_WHEEL_REVS_PER_MOTOR_REV));
            telemetry.addLine("A: reset field heading");
            telemetry.update();
        }

        stopDrive();
    }

    private void configureForVelocityControl(DcMotorEx motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /** Converts a desired wheel RPM into the motor encoder ticks/second expected by setVelocity. */
    private void setWheelRpm(DcMotorEx motor, double wheelRpm,
                             double wheelRevsPerMotorRev) {
        double motorRpm = wheelRpm / wheelRevsPerMotorRev;
        double ticksPerSecond = motorRpm * motor.getMotorType().getTicksPerRev() / 60.0;
        motor.setVelocity(ticksPerSecond);
    }

    /** Converts measured motor encoder velocity back into wheel RPM for useful telemetry. */
    private double getWheelRpm(DcMotorEx motor, double wheelRevsPerMotorRev) {
        double motorRpm = motor.getVelocity() * 60.0 / motor.getMotorType().getTicksPerRev();
        return motorRpm * wheelRevsPerMotorRev;
    }

    private double minimumAchievableWheelRpm() {
        double frontLeftMax = frontLeft.getMotorType().getMaxRPM()
                * frontWheelRevsPerMotorRev;
        double frontRightMax = frontRight.getMotorType().getMaxRPM()
                * frontWheelRevsPerMotorRev;
        double backLeftMax = backLeft.getMotorType().getMaxRPM()
                * REAR_WHEEL_REVS_PER_MOTOR_REV;
        double backRightMax = backRight.getMotorType().getMaxRPM()
                * REAR_WHEEL_REVS_PER_MOTOR_REV;

        return Math.min(
                Math.min(frontLeftMax, frontRightMax),
                Math.min(backLeftMax, backRightMax));
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < JOYSTICK_DEADBAND ? 0.0 : value;
    }

    private void stopDrive() {
        frontLeft.setVelocity(0.0);
        frontRight.setVelocity(0.0);
        backLeft.setVelocity(0.0);
        backRight.setVelocity(0.0);
    }

    private void validateConstants() {
        if (FRONT_MOTOR_SPROCKET_TEETH <= 0.0 || FRONT_WHEEL_SPROCKET_TEETH <= 0.0) {
            throw new IllegalArgumentException("Front sprocket tooth counts must be positive.");
        }
        if (REAR_WHEEL_REVS_PER_MOTOR_REV <= 0.0) {
            throw new IllegalArgumentException("Rear wheel ratio must be positive.");
        }
        if (MAX_SPEED_FRACTION <= 0.0 || MAX_SPEED_FRACTION > 1.0) {
            throw new IllegalArgumentException("MAX_SPEED_FRACTION must be in (0, 1].");
        }
        if (JOYSTICK_DEADBAND < 0.0 || JOYSTICK_DEADBAND >= 1.0) {
            throw new IllegalArgumentException("JOYSTICK_DEADBAND must be in [0, 1).");
        }
    }
}
