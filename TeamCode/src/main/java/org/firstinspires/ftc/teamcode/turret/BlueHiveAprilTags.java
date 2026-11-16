package org.firstinspires.ftc.teamcode.turret;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import java.util.Locale;

/**
 * Tracks blue BIOBUZZ AprilTags with a turret-mounted Limelight 3A.
 *
 * Standalone subsystem, NOT an OpMode. Call start(), update(), addTelemetry(),
 * and stop() from the eventual TeleOp.
 *
 * When the direct aiming angle is beyond the mechanical limits, this class
 * tries an equivalent angle +/- 360 degrees. That alternative is possible
 * ONLY when the turret's actual geared travel includes that angle.
 * A positional servo cannot rotate through its mechanical end stops.
 *
 * No encoder is used: all turret angles are estimates from servo commands.
 * Set the external gearing ratio, direction and home alignment correctly.
 */
public class BlueHiveAprilTags {

    public enum TargetCell {
        ANY_BLUE,               // IDs 38-45
        AUDIENCE_SIDE,          // IDs 38-41
        OPPOSITE_AUDIENCE_SIDE  // IDs 42-45
    }

    private static final String SERVO_NAME = "turret";
    private static final String LIMELIGHT_NAME = "limelight";
    private static final int DEFAULT_APRILTAG_PIPELINE = 0; // Change to your 36h11 pipeline.

    // Positional goBILDA 2000-0025-0004: nominally 300 servo degrees across 500-2500 us.
    private static final double SERVO_TRAVEL_DEGREES = 300.0;
    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;
    private static final double SERVO_HOME = 0.5;

    // ONLY EDIT THIS NUMBER FOR EXTERNAL GEARING.
    // Gear ratio = TURRET degrees of rotation / SERVO output degrees of rotation.
    // Example: turret rotates 5.4 degrees per 3.5 degrees of servo rotation.
    // 5.4 / 3.5 gives a turret speed multiplier of about 1.54x.
    // Do NOT include the servo's built-in gearbox ratio.
    private static final double GEAR_RATIO = 5.4 / 3.5;

    // +1 if increasing servo position turns the camera clockwise (right);
    // -1 if increasing servo position turns the camera counterclockwise (left).
    private static final double DIRECTION_SIGN = 1.0;

    private static final double CAMERA_AIM_OFFSET_DEGREES = 0.0;
    private static final double TRACKING_KP = 0.45;
    private static final double AIM_TOLERANCE_DEGREES = 1.25;
    private static final double MAX_TURRET_CORRECTION_DEGREES = 10.0;
    private static final double MAX_SERVO_SPEED_DEGREES_PER_SEC = 240.0;
    private static final double TAG_LOST_TIMEOUT_SECONDS = 0.40;
    private static final long MAX_FRAME_STALENESS_MS = 120;

    // When a wrap-around path is selected, continue along it even after the
    // AprilTag leaves the camera view. Give it time to become visible again.
    private static final double WRAP_REACQUIRE_WINDOW_DEGREES = 12.0;
    private static final double WRAP_ARRIVAL_SERVO_TOLERANCE = 0.008;
    private static final double WRAP_REACQUIRE_WAIT_SECONDS = 0.85;
    private static final double WRAP_MAX_SECONDS = 4.0;
    private static final double FULL_TURN_DEGREES = 360.0;
    // Only take the long way around if the target is MORE than this far past a
    // limit; a target just past the end stop is handled by clamping instead.
    private static final double WRAP_TRIGGER_MARGIN_DEGREES = 5.0;
    private static final double WRAP_FAILED_STATUS_SECONDS = 1.5;

    private final Servo turretServo;
    private final Limelight3A limelight;
    private final int aprilTagPipeline;

    private TargetCell targetCell = TargetCell.ANY_BLUE;
    private boolean started = false;
    private double commandedServoPosition = SERVO_HOME;
    private double targetServoPosition = SERVO_HOME;
    private double lastFrameTimestamp = Double.NaN;
    private long lastUpdateNanos = 0L;
    private long lastSeenNanos = 0L;
    private int trackedTagId = -1;
    private double txDegrees = Double.NaN;
    private String trackingStatus = "NOT STARTED";

    private boolean wrapping = false;
    private long wrapStartedNanos = 0L;
    private long wrapArrivedNanos = 0L;
    private long wrapFailedNanos = 0L;
    private int wrapTagId = -1;
    private double wrapTargetAngleDegrees = Double.NaN;

    public BlueHiveAprilTags(HardwareMap hardwareMap) {
        this(hardwareMap, DEFAULT_APRILTAG_PIPELINE);
    }

    public BlueHiveAprilTags(HardwareMap hardwareMap, int aprilTagPipeline) {
        if (aprilTagPipeline < 0) {
            throw new IllegalArgumentException("AprilTag pipeline index cannot be negative.");
        }
        if (!Double.isFinite(GEAR_RATIO)
                || GEAR_RATIO <= 0.0
                || (DIRECTION_SIGN != 1.0 && DIRECTION_SIGN != -1.0)
                || SERVO_HOME < SERVO_MIN || SERVO_HOME > SERVO_MAX) {
            throw new IllegalArgumentException("Invalid turret gearing, direction or home.");
        }

        this.aprilTagPipeline = aprilTagPipeline;
        turretServo = hardwareMap.get(Servo.class, SERVO_NAME);
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);

        turretServo.setDirection(Servo.Direction.FORWARD);
        if (turretServo instanceof ServoImplEx) {
            ((ServoImplEx) turretServo).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
        // Otherwise configure a full-range servo in FTC Robot Configuration.
    }

    /** Starts tracking and homes the servo; verify mechanical clearance first. */
    public void start() {
        started = true;
        commandedServoPosition = SERVO_HOME;
        targetServoPosition = SERVO_HOME;
        lastFrameTimestamp = Double.NaN;
        lastUpdateNanos = System.nanoTime();
        lastSeenNanos = 0L;
        trackedTagId = -1;
        txDegrees = Double.NaN;
        resetWrap();
        trackingStatus = "SEARCHING";
        wrapFailedNanos = 0L;
        turretServo.setPosition(SERVO_HOME);
        limelight.pipelineSwitch(aprilTagPipeline);
        limelight.start();
    }

    /** Call once per TeleOp loop. */
    public void update() {
        if (!started) {
            return;
        }

        long now = System.nanoTime();
        double dt = Math.min(0.10, Math.max(0.0,
                (now - lastUpdateNanos) / 1_000_000_000.0));
        lastUpdateNanos = now;

        LLResult result = limelight.getLatestResult();

        // Avoid interfering with later classifiers or other Limelight pipelines.
        if (result != null && result.getPipelineIndex() != aprilTagPipeline) {
            resetWrap();
            targetServoPosition = commandedServoPosition;
            lastSeenNanos = 0L;
            trackedTagId = -1;
            txDegrees = Double.NaN;
            trackingStatus = "OTHER PIPELINE ACTIVE";
            return;
        }

        boolean freshFrame = result != null && result.isValid()
                && result.getStaleness() <= MAX_FRAME_STALENESS_MS
                && Double.isFinite(result.getTimestamp())
                && Double.compare(result.getTimestamp(), lastFrameTimestamp) != 0;

        if (freshFrame) {
            lastFrameTimestamp = result.getTimestamp();
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

            if (wrapping) {
                // Don't switch to another tag mid-wrap. The camera will lose
                // the original tag while traversing the longer route.
                LLResultTypes.FiducialResult lockedTag = findTag(tags, wrapTagId);
                if (lockedTag != null) {
                    txDegrees = lockedTag.getTargetXDegrees();
                    lastSeenNanos = now;
                    if (wrapArrivedNanos != 0L
                            || Math.abs(txDegrees - CAMERA_AIM_OFFSET_DEGREES)
                            <= WRAP_REACQUIRE_WINDOW_DEGREES) {
                        // Original tag is visible in roughly the correct direction:
                        // hand control back to the standard fine-aiming controller.
                        resetWrap();
                        aimAtTag(lockedTag, now);
                    }
                }
            } else {
                LLResultTypes.FiducialResult tag = chooseBlueTag(tags);
                if (tag != null) {
                    aimAtTag(tag, now);
                } else {
                    trackingStatus = "TAG TEMPORARILY LOST";
                }
            }
        }

        if (wrapping) {
            // Never return home merely because the tag is out of the camera's
            // view during the chosen wrap-around route.
            if (Math.abs(commandedServoPosition - targetServoPosition)
                    <= WRAP_ARRIVAL_SERVO_TOLERANCE && wrapArrivedNanos == 0L) {
                wrapArrivedNanos = now;
            }

            boolean timedOut = now - wrapStartedNanos >= secondsToNanos(WRAP_MAX_SECONDS);
            boolean failedToReacquire = wrapArrivedNanos != 0L &&
                    now - wrapArrivedNanos >= secondsToNanos(WRAP_REACQUIRE_WAIT_SECONDS);
            if (timedOut || failedToReacquire) {
                resetWrap();
                lastSeenNanos = 0L;
                trackedTagId = -1;
                txDegrees = Double.NaN;
                targetServoPosition = SERVO_HOME;
                trackingStatus = "WRAP FAILED - RETURNING HOME";
                wrapFailedNanos = now;
            } else {
                trackingStatus = wrapArrivedNanos != 0L
                        ? "WRAPPED - REACQUIRING TAG" : "WRAPPING INSIDE LIMITS";
            }
        } else {
            boolean recentlySeen = lastSeenNanos != 0L &&
                    now - lastSeenNanos < secondsToNanos(TAG_LOST_TIMEOUT_SECONDS);
            if (!recentlySeen) {
                trackedTagId = -1;
                txDegrees = Double.NaN;
                targetServoPosition = SERVO_HOME;
                boolean atHome = Math.abs(commandedServoPosition - SERVO_HOME) < 0.005;
                if (wrapFailedNanos != 0L
                        && now - wrapFailedNanos < secondsToNanos(WRAP_FAILED_STATUS_SECONDS)) {
                    trackingStatus = "WRAP FAILED - RETURNING HOME";
                } else {
                    trackingStatus = atHome ? "SEARCHING" : "RETURNING HOME";
                }
            }
        }

        double maxStep = MAX_SERVO_SPEED_DEGREES_PER_SEC * dt / SERVO_TRAVEL_DEGREES;
        commandedServoPosition = moveTowards(commandedServoPosition,
                clip(targetServoPosition, SERVO_MIN, SERVO_MAX), maxStep);
        turretServo.setPosition(commandedServoPosition);
    }

    /** Process the camera bearing (positive tx means target to camera right). */
    private void aimAtTag(LLResultTypes.FiducialResult tag, long now) {
        trackedTagId = tag.getFiducialId();
        txDegrees = tag.getTargetXDegrees();
        lastSeenNanos = now;

        double errorDegrees = txDegrees - CAMERA_AIM_OFFSET_DEGREES;
        if (Math.abs(errorDegrees) <= AIM_TOLERANCE_DEGREES) {
            targetServoPosition = commandedServoPosition;
            trackingStatus = "ALIGNED";
            return;
        }

        double currentTurretAngle = servoToTurretAngle(commandedServoPosition);
        double desiredTurretAngle = currentTurretAngle + errorDegrees;
        double endA = servoToTurretAngle(SERVO_MIN);
        double endB = servoToTurretAngle(SERVO_MAX);
        double minAngle = Math.min(endA, endB);
        double maxAngle = Math.max(endA, endB);

        // Only take the long way around when the direct target is OUT of bounds.
        // This handles cases such as +200 degrees being equivalent to -160.
        if (desiredTurretAngle < minAngle || desiredTurretAngle > maxAngle) {
            double overshoot = Math.max(minAngle - desiredTurretAngle,
                    desiredTurretAngle - maxAngle);
            double alternateAngle = overshoot > WRAP_TRIGGER_MARGIN_DEGREES
                    ? findReachableWrappedAngle(desiredTurretAngle,
                    currentTurretAngle, minAngle, maxAngle)
                    : Double.NaN;
            if (Double.isFinite(alternateAngle)) {
                wrapping = true;
                wrapStartedNanos = now;
                wrapArrivedNanos = 0L;
                wrapTagId = trackedTagId;
                wrapTargetAngleDegrees = alternateAngle;
                targetServoPosition = clip(turretAngleToServo(alternateAngle), SERVO_MIN, SERVO_MAX);
                trackingStatus = "WRAPPING INSIDE LIMITS";
                return;
            }

            // Neither the direct angle nor a 360-degree equivalent is reachable.
            // Move toward the nearest possible boundary without exceeding it.
            double safeAngle = clip(desiredTurretAngle, minAngle, maxAngle);
            double correction = clip(TRACKING_KP * (safeAngle - currentTurretAngle),
                    -MAX_TURRET_CORRECTION_DEGREES, MAX_TURRET_CORRECTION_DEGREES);
            targetServoPosition = clip(turretAngleToServo(currentTurretAngle + correction),
                    SERVO_MIN, SERVO_MAX);
            trackingStatus = "TARGET OUT OF REACH";
            return;
        }

        double correctionDegrees = clip(TRACKING_KP * errorDegrees,
                -MAX_TURRET_CORRECTION_DEGREES, MAX_TURRET_CORRECTION_DEGREES);
        targetServoPosition = clip(turretAngleToServo(currentTurretAngle + correctionDegrees),
                SERVO_MIN, SERVO_MAX);
        trackingStatus = "TRACKING";
    }

    /**
     * Find desiredAngle + 360*k within the mechanical interval (k != 0).
     * Choose the one that requires the least physical movement from currentAngle.
     * Returns NaN if even the long route is not mechanically reachable.
     */
    private static double findReachableWrappedAngle(double desiredAngle, double currentAngle,
                                                    double minAngle, double maxAngle) {
        long minK = (long) Math.ceil((minAngle - desiredAngle) / FULL_TURN_DEGREES);
        long maxK = (long) Math.floor((maxAngle - desiredAngle) / FULL_TURN_DEGREES);
        if (minK > maxK) {
            return Double.NaN;
        }
        long closestK = Math.round((currentAngle - desiredAngle) / FULL_TURN_DEGREES);
        long chosenK = Math.max(minK, Math.min(maxK, closestK));
        return chosenK == 0L ? Double.NaN : desiredAngle + chosenK * FULL_TURN_DEGREES;
    }

    /** Increases in this angle correspond to looking right from the robot. */
    private static double servoToTurretAngle(double servoPosition) {
        return DIRECTION_SIGN * (servoPosition - SERVO_HOME)
                * SERVO_TRAVEL_DEGREES * GEAR_RATIO;
    }

    private static double turretAngleToServo(double turretAngleDegrees) {
        return SERVO_HOME + DIRECTION_SIGN * turretAngleDegrees
                / (SERVO_TRAVEL_DEGREES * GEAR_RATIO);
    }

    /** Optional: restrict tracking to one of the two blue cells. */
    public void setTargetCell(TargetCell cell) {
        if (cell == null) {
            throw new IllegalArgumentException("Target cell cannot be null.");
        }
        targetCell = cell;
        resetWrap();
        targetServoPosition = commandedServoPosition;
        trackedTagId = -1;
        txDegrees = Double.NaN;
        lastSeenNanos = 0L;
    }

    public boolean hasRecentBlueTag() {
        return started && lastSeenNanos != 0L &&
                System.nanoTime() - lastSeenNanos < secondsToNanos(TAG_LOST_TIMEOUT_SECONDS);
    }

    public int getTrackedTagId() {
        return trackedTagId;
    }

    public void addInitializationTelemetry(Telemetry telemetry) {
        telemetry.addLine("Blue AprilTag turret: auto tracking with boundary-aware aiming.");
        telemetry.addLine("Check external gearing, servo direction, home, and 36h11 pipeline.");
        addTelemetry(telemetry);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret status", trackingStatus);
        telemetry.addData("Blue cell", targetCell);
        telemetry.addData("AprilTag pipeline", aprilTagPipeline);
        telemetry.addData("Locked tag", trackedTagId < 0 ? "none" : trackedTagId);
        telemetry.addData("AprilTag tx (deg)",
                Double.isFinite(txDegrees) ? String.format(Locale.US, "%.2f", txDegrees) : "--");
        telemetry.addData("Turret gearing (turret/servo)", "%.4f", GEAR_RATIO);
        telemetry.addData("Turret angle est (deg)", "%.1f",
                servoToTurretAngle(commandedServoPosition));
        telemetry.addData("Turret total travel (deg)", "%.1f",
                SERVO_TRAVEL_DEGREES * (SERVO_MAX - SERVO_MIN) * GEAR_RATIO);
        telemetry.addData("Wrap route active", wrapping);
        if (wrapping) {
            telemetry.addData("Wrap destination (deg)", "%.1f", wrapTargetAngleDegrees);
        }
        telemetry.addData("Turret servo command", "%.3f", commandedServoPosition);
        telemetry.addData("Turret servo target", "%.3f", targetServoPosition);
        telemetry.addData("At servo limit",
                commandedServoPosition <= SERVO_MIN + 0.001
                        || commandedServoPosition >= SERVO_MAX - 0.001);
    }

    /** Stops Limelight polling; leaves the turret in its last commanded position. */
    public void stop() {
        started = false;
        resetWrap();
        trackingStatus = "STOPPED";
        limelight.stop();
    }

    private void resetWrap() {
        wrapping = false;
        wrapStartedNanos = 0L;
        wrapArrivedNanos = 0L;
        wrapTagId = -1;
        wrapTargetAngleDegrees = Double.NaN;
    }

    private LLResultTypes.FiducialResult chooseBlueTag(
            List<LLResultTypes.FiducialResult> tags) {
        if (tags == null || tags.isEmpty()) {
            return null;
        }
        LLResultTypes.FiducialResult best = null;
        for (LLResultTypes.FiducialResult tag : tags) {
            if (tag == null || !isAllowedTag(tag.getFiducialId())
                    || !Double.isFinite(tag.getTargetXDegrees())) {
                continue;
            }
            if (tag.getFiducialId() == trackedTagId) {
                return tag;
            }
            if (best == null || Math.abs(tag.getTargetXDegrees())
                    < Math.abs(best.getTargetXDegrees())) {
                best = tag;
            }
        }
        return best;
    }

    private LLResultTypes.FiducialResult findTag(
            List<LLResultTypes.FiducialResult> tags, int id) {
        if (tags == null) {
            return null;
        }
        for (LLResultTypes.FiducialResult tag : tags) {
            if (tag != null && tag.getFiducialId() == id && isAllowedTag(id)
                    && Double.isFinite(tag.getTargetXDegrees())) {
                return tag;
            }
        }
        return null;
    }

    private boolean isAllowedTag(int id) {
        switch (targetCell) {
            case AUDIENCE_SIDE:
                return id >= 38 && id <= 41;
            case OPPOSITE_AUDIENCE_SIDE:
                return id >= 42 && id <= 45;
            case ANY_BLUE:
            default:
                return id >= 38 && id <= 45;
        }
    }

    private static long secondsToNanos(double seconds) {
        return (long) (seconds * 1_000_000_000.0);
    }

    private static double moveTowards(double current, double target, double maxChange) {
        return current + clip(target - current, -maxChange, maxChange);
    }

    private static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}