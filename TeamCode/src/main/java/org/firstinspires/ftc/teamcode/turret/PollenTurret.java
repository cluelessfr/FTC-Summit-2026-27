package org.firstinspires.ftc.teamcode.turret;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.turret.HiveOpeningAim.CameraMount;
import org.firstinspires.ftc.teamcode.turret.HiveOpeningAim.OpeningAim;
import org.firstinspires.ftc.teamcode.turret.HiveOpeningAim.Vector;

import java.util.List;
import java.util.Locale;

/**
 * Aims POLLEN at the upward blue BIOBUZZ CELL and provides both launcher bearings.
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
public final class PollenTurret {

    public enum TargetCell {
        ANY_BLUE,               // IDs 38-45
        AUDIENCE_SIDE,          // IDs 38-41
        OPPOSITE_AUDIENCE_SIDE  // IDs 42-45
    }

    private static final String SERVO_NAME = "pollenTurret";
    private static final String LIMELIGHT_NAME = "limelight";
    private static final int DEFAULT_APRILTAG_PIPELINE = 0; // Change to your 36h11 pipeline.

    // Verify the servo home, limits, travel, gearing, and direction on the robot.
    private static final double SERVO_HOME = 0.5;
    private static final double SERVO_MIN = 0;
    private static final double SERVO_MAX = 1;
    private static final double SERVO_TRAVEL_DEGREES = 300;
    private static final double TURRET_DEGREES_PER_SERVO_DEGREE = 5.4 / 3.5;
    private static final double SERVO_RIGHT_SIGN = 1; // +1 if increasing position turns right.
    private static final TurretServo.Calibration DEFAULT_CALIBRATION =
            new TurretServo.Calibration(SERVO_HOME, SERVO_MIN, SERVO_MAX, SERVO_TRAVEL_DEGREES,
                    TURRET_DEGREES_PER_SERVO_DEGREE, SERVO_RIGHT_SIGN);

    // Measure the lens from the POLLEN rotation axis with the turret at home.
    private static final double LENS_FORWARD_METERS = -0.035;
    private static final double LENS_LEFT_METERS = 0;
    private static final double LENS_UP_METERS = 0.1;
    // Positive yaw turns left, positive pitch points up, and positive roll raises the left side.
    private static final double CAMERA_YAW_DEGREES = 0;
    private static final double CAMERA_PITCH_UP_DEGREES = 50;
    private static final double CAMERA_ROLL_DEGREES = 0;
    private static final CameraMount DEFAULT_CAMERA_MOUNT = new CameraMount(
            new Vector(LENS_FORWARD_METERS, LENS_LEFT_METERS, LENS_UP_METERS),
            Math.toRadians(CAMERA_YAW_DEGREES), Math.toRadians(CAMERA_PITCH_UP_DEGREES),
            Math.toRadians(CAMERA_ROLL_DEGREES));
    private static final Vector POLLEN_AXIS_ORIGIN = new Vector(0, 0, 0);

    private static final double TRACKING_KP = 0.45;
    private static final double AIM_TOLERANCE_DEGREES = 1.25;
    private static final double MAX_TURRET_CORRECTION_DEGREES = 10.0;
    private static final double MAX_SERVO_SPEED_DEGREES_PER_SEC = 240.0;
    private static final double TAG_LOST_TIMEOUT_SECONDS = 0.40;
    private static final long MAX_FRAME_STALENESS_MS = 120;
    private static final long POLLEN_SETTLE_MS = 250;
    private static final long HOME_SETTLE_MS = 1000;

    // When a wrap-around path is selected, continue along it even after the
    // AprilTag leaves the camera view. Give it time to become visible again.
    private static final double WRAP_ARRIVAL_SERVO_TOLERANCE = 0.008;
    private static final double WRAP_REACQUIRE_WAIT_SECONDS = 0.85;
    private static final double WRAP_MAX_SECONDS = 4.0;
    private static final double FULL_TURN_DEGREES = 360.0;
    // Only take the long way around if the target is MORE than this far past a
    // limit; a target just past the end stop is handled by clamping instead.
    private static final double WRAP_TRIGGER_MARGIN_DEGREES = 5.0;
    private static final double WRAP_FAILED_STATUS_SECONDS = 1.5;

    private final TurretServo servo;
    private final TurretServo.Calibration calibration;
    private final Limelight3A limelight;
    private final int aprilTagPipeline;

    private CameraMount cameraMount;
    private Vector pollenAxis;
    private Vector nectarAxis;
    private OpeningAim openingAim;
    private long pollenSettlesAtNanos;
    private TargetCell targetCell = TargetCell.ANY_BLUE;
    private boolean started = false;
    private double commandedServoPosition;
    private double targetServoPosition;
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

    public PollenTurret(HardwareMap hardwareMap) {
        this(hardwareMap, DEFAULT_APRILTAG_PIPELINE);
    }

    public PollenTurret(HardwareMap hardwareMap, int aprilTagPipeline) {
        this(hardwareMap, aprilTagPipeline, DEFAULT_CALIBRATION);
    }

    public PollenTurret(HardwareMap hardwareMap, int aprilTagPipeline,
                        TurretServo.Calibration calibration) {
        if (aprilTagPipeline < 0 || calibration == null) {
            throw new IllegalArgumentException("Set a valid pipeline and POLLEN calibration.");
        }
        this.aprilTagPipeline = aprilTagPipeline;
        this.calibration = calibration;
        commandedServoPosition = calibration.home;
        targetServoPosition = calibration.home;
        servo = new TurretServo(hardwareMap, SERVO_NAME);
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        if (hasDefaultCameraMeasurements()) {
            // POLLEN-only aiming uses the POLLEN pivot as the origin for both bearings.
            configureOpeningAim(DEFAULT_CAMERA_MOUNT, POLLEN_AXIS_ORIGIN, POLLEN_AXIS_ORIGIN);
        }
    }

    public static boolean hasDefaultCameraMeasurements() {
        return isValidMount(DEFAULT_CAMERA_MOUNT);
    }

    private static boolean isValidMount(CameraMount mount) {
        return mount != null && mount.lensFromPollenAxis != null
                && mount.lensFromPollenAxis.isFinite()
                && Double.isFinite(mount.yaw) && Double.isFinite(mount.pitch) && Double.isFinite(mount.roll);
    }

    /** Override the default mount and turret positions before start. Positions are in meters. */
    public void configureOpeningAim(CameraMount cameraMount, Vector pollenAxis, Vector nectarAxis) {
        if (started) throw new IllegalStateException("Configure the aim before starting.");
        if (!isValidMount(cameraMount) || pollenAxis == null || nectarAxis == null
                || !pollenAxis.isFinite() || !nectarAxis.isFinite()) {
            throw new IllegalArgumentException("Set a finite camera mount and both turret positions.");
        }
        this.cameraMount = cameraMount;
        this.pollenAxis = pollenAxis;
        this.nectarAxis = nectarAxis;
    }

    /** Starts tracking and homes the servo; verify mechanical clearance first. */
    public void start() {
        if (cameraMount == null) {
            trackingStatus = "CONFIGURE CAMERA MOUNT AND TURRET AXES";
            return;
        }
        started = true;
        commandedServoPosition = calibration.home;
        targetServoPosition = calibration.home;
        lastFrameTimestamp = Double.NaN;
        lastUpdateNanos = System.nanoTime();
        pollenSettlesAtNanos = lastUpdateNanos + HOME_SETTLE_MS * 1_000_000L;
        lastSeenNanos = 0L;
        openingAim = null;
        trackedTagId = -1;
        txDegrees = Double.NaN;
        resetWrap();
        trackingStatus = "SEARCHING";
        wrapFailedNanos = 0L;
        servo.setPosition(calibration.home);
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
            openingAim = null;
            trackedTagId = -1;
            txDegrees = Double.NaN;
            trackingStatus = "OTHER PIPELINE ACTIVE";
            return;
        }

        long frameAgeMs = result == null ? -1 : result.getStaleness();
        boolean freshFrame = frameAgeMs >= 0 && frameAgeMs <= MAX_FRAME_STALENESS_MS
                && Double.isFinite(result.getTimestamp())
                && Double.compare(result.getTimestamp(), lastFrameTimestamp) != 0;

        if (freshFrame) {
            lastFrameTimestamp = result.getTimestamp();
            double delayMs = frameAgeMs + result.getCaptureLatency() + result.getTargetingLatency();
            // The camera moves with POLLEN. Use only exposures taken after the last
            // command settled, so its commanded yaw is also the yaw at capture.
            if (Double.isFinite(delayMs) && delayMs >= 0
                    && now - delayMs * 1_000_000 >= pollenSettlesAtNanos) {
                if (!result.isValid() || !aimFromTags(result.getFiducialResults(), now)) {
                    openingAim = null;
                    if (!wrapping) trackingStatus = "NO STABLE CELL OPENING";
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
                openingAim = null;
                trackedTagId = -1;
                txDegrees = Double.NaN;
                targetServoPosition = calibration.home;
                trackingStatus = "WRAP FAILED - RETURNING HOME";
                wrapFailedNanos = now;
            } else {
                trackingStatus = wrapArrivedNanos != 0L
                        ? "WRAPPED - REACQUIRING TAG" : "WRAPPING INSIDE LIMITS";
            }
        } else {
            boolean recentlySeen = lastSeenNanos != 0L &&
                    now - lastSeenNanos < secondsToNanos(TAG_LOST_TIMEOUT_SECONDS);
            if (!recentlySeen && now >= pollenSettlesAtNanos + MAX_FRAME_STALENESS_MS * 1_000_000L) {
                trackedTagId = -1;
                txDegrees = Double.NaN;
                openingAim = null;
                targetServoPosition = calibration.home;
                boolean atHome = Math.abs(commandedServoPosition - calibration.home) < 0.005;
                if (wrapFailedNanos != 0L
                        && now - wrapFailedNanos < secondsToNanos(WRAP_FAILED_STATUS_SECONDS)) {
                    trackingStatus = "WRAP FAILED - RETURNING HOME";
                } else {
                    trackingStatus = atHome ? "SEARCHING" : "RETURNING HOME";
                }
            }
        }

        double maxStep = MAX_SERVO_SPEED_DEGREES_PER_SEC * dt / calibration.travelDegrees;
        double nextPosition = TurretServo.moveTowards(commandedServoPosition,
                calibration.limit(targetServoPosition), maxStep);
        if (nextPosition != commandedServoPosition) {
            commandedServoPosition = nextPosition;
            servo.setPosition(commandedServoPosition);
            pollenSettlesAtNanos = now + POLLEN_SETTLE_MS * 1_000_000L;
        }
    }

    private void aimAtOpening(OpeningAim aim, long now) {
        trackedTagId = aim.tagId;
        openingAim = aim;
        lastSeenNanos = now;

        double currentTurretAngle = calibration.rightDegrees(commandedServoPosition);
        double errorDegrees = openingErrorDegrees(aim.pollenYaw, currentTurretAngle);
        if (Math.abs(errorDegrees) <= AIM_TOLERANCE_DEGREES) {
            targetServoPosition = commandedServoPosition;
            trackingStatus = "ALIGNED";
            return;
        }

        double desiredTurretAngle = currentTurretAngle + errorDegrees;
        double endA = calibration.rightDegrees(calibration.min);
        double endB = calibration.rightDegrees(calibration.max);
        double minAngle = Math.min(endA, endB);
        double maxAngle = Math.max(endA, endB);

        // Only take the long way around when the direct target is OUT of bounds.
        // This handles cases such as +200 degrees being equivalent to -160.
        if (desiredTurretAngle < minAngle || desiredTurretAngle > maxAngle) {
            double overshoot = Math.max(minAngle - desiredTurretAngle,
                    desiredTurretAngle - maxAngle);
            double alternateAngle = overshoot > WRAP_TRIGGER_MARGIN_DEGREES
                    ? calibration.nearestReachableRightDegrees(desiredTurretAngle, currentTurretAngle)
                    : Double.NaN;
            if (Double.isFinite(alternateAngle)) {
                wrapping = true;
                wrapStartedNanos = now;
                wrapArrivedNanos = 0L;
                wrapTagId = trackedTagId;
                wrapTargetAngleDegrees = alternateAngle;
                targetServoPosition = calibration.limit(calibration.position(alternateAngle));
                trackingStatus = "WRAPPING INSIDE LIMITS";
                return;
            }

            // Neither the direct angle nor a 360-degree equivalent is reachable.
            // Move toward the nearest possible boundary without exceeding it.
            double safeAngle = clip(desiredTurretAngle, minAngle, maxAngle);
            double correction = clip(TRACKING_KP * (safeAngle - currentTurretAngle),
                    -MAX_TURRET_CORRECTION_DEGREES, MAX_TURRET_CORRECTION_DEGREES);
            targetServoPosition = calibration.limit(calibration.position(currentTurretAngle + correction));
            trackingStatus = "TARGET OUT OF REACH";
            return;
        }

        double correctionDegrees = clip(TRACKING_KP * errorDegrees,
                -MAX_TURRET_CORRECTION_DEGREES, MAX_TURRET_CORRECTION_DEGREES);
        targetServoPosition = calibration.limit(calibration.position(currentTurretAngle + correctionDegrees));
        trackingStatus = "TRACKING";
    }

    static double openingErrorDegrees(double openingYawRadians, double currentRightDegrees) {
        // The geometry reports positive left. The servo mapping uses positive right.
        return Math.IEEEremainder(-Math.toDegrees(openingYawRadians) - currentRightDegrees,
                FULL_TURN_DEGREES);
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
        openingAim = null;
    }

    public boolean hasRecentBlueTag() {
        return started && lastSeenNanos != 0L &&
                System.nanoTime() - lastSeenNanos < secondsToNanos(TAG_LOST_TIMEOUT_SECONDS);
    }

    public int getTrackedTagId() {
        return trackedTagId;
    }

    /** Latest usable target and both bearings, with positive yaw pointing left. */
    public OpeningAim getOpeningAim() {
        return hasRecentBlueTag() ? openingAim : null;
    }

    public void addInitializationTelemetry(Telemetry telemetry) {
        telemetry.addLine("Blue AprilTag turret: aims at the upward CELL opening.");
        telemetry.addLine("Check camera mount, turret axes, servo calibration, and 3D pipeline.");
        addTelemetry(telemetry);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret status", trackingStatus);
        telemetry.addData("Blue cell", targetCell);
        telemetry.addData("AprilTag pipeline", aprilTagPipeline);
        telemetry.addData("Locked tag", trackedTagId < 0 ? "none" : trackedTagId);
        telemetry.addData("AprilTag tx (deg)",
                Double.isFinite(txDegrees) ? String.format(Locale.US, "%.2f", txDegrees) : "--");
        OpeningAim aim = getOpeningAim();
        if (aim != null) {
            telemetry.addData("CELL up", aim.audienceCellUp ? "audience" : "opposite audience");
            telemetry.addData("CELL aim point forward/left/up (m)", "%.3f / %.3f / %.3f",
                    aim.opening.x, aim.opening.y, aim.opening.z);
            telemetry.addData("POLLEN / NECTAR yaw (deg, +left)", "%.2f / %.2f",
                    Math.toDegrees(aim.pollenYaw), Math.toDegrees(aim.nectarYaw));
        }
        telemetry.addData("Turret gearing (turret/servo)", "%.4f", calibration.turretPerServoDegree);
        telemetry.addData("Turret angle est (deg)", "%.1f",
                calibration.rightDegrees(commandedServoPosition));
        telemetry.addData("Turret total travel (deg)", "%.1f",
                calibration.travelDegrees * (calibration.max - calibration.min) * calibration.turretPerServoDegree);
        telemetry.addData("Wrap route active", wrapping);
        if (wrapping) {
            telemetry.addData("Wrap destination (deg)", "%.1f", wrapTargetAngleDegrees);
        }
        telemetry.addData("Turret servo command", "%.3f", commandedServoPosition);
        telemetry.addData("Turret servo target", "%.3f", targetServoPosition);
        telemetry.addData("At servo limit",
                commandedServoPosition <= calibration.min + 0.001
                        || commandedServoPosition >= calibration.max - 0.001);
    }

    /** Stops Limelight polling; leaves the turret in its last commanded position. */
    public void stop() {
        started = false;
        openingAim = null;
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

    private boolean aimFromTags(List<LLResultTypes.FiducialResult> tags, long now) {
        if (tags == null || (wrapping && wrapArrivedNanos == 0L)) return false;
        double pollenYaw = Math.toRadians(-calibration.rightDegrees(commandedServoPosition));
        OpeningAim best = null;
        LLResultTypes.FiducialResult bestTag = null;
        for (LLResultTypes.FiducialResult tag : tags) {
            if (tag == null || !isAllowedTag(tag.getFiducialId())) continue;
            if (wrapping && tag.getFiducialId() != wrapTagId) continue;
            OpeningAim aim = HiveOpeningAim.estimateOpening(HiveOpeningAim.Alliance.BLUE,
                    tag, cameraMount, pollenYaw, pollenAxis, nectarAxis);
            if (aim == null || !Double.isFinite(aim.pollenYaw) || !Double.isFinite(aim.nectarYaw)) {
                continue;
            }
            if (bestTag == null || tag.getTargetArea() > bestTag.getTargetArea()
                    || tag.getFiducialId() == trackedTagId) {
                best = aim;
                bestTag = tag;
            }
            if (tag.getFiducialId() == trackedTagId) break;
        }
        if (best == null) return false;
        if (wrapping) resetWrap();
        txDegrees = bestTag.getTargetXDegrees();
        aimAtOpening(best, now);
        return true;
    }

    private boolean isAllowedTag(int id) {
        HiveOpeningAim.Alliance alliance = HiveOpeningAim.Alliance.BLUE;
        if (!alliance.accepts(id)) return false;
        switch (targetCell) {
            case AUDIENCE_SIDE:
                return alliance.isAudienceTag(id);
            case OPPOSITE_AUDIENCE_SIDE:
                return !alliance.isAudienceTag(id);
            case ANY_BLUE:
            default:
                return true;
        }
    }

    private static long secondsToNanos(double seconds) {
        return (long) (seconds * 1_000_000_000.0);
    }

    private static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
