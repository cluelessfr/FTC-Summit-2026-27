package org.firstinspires.ftc.teamcode.turret;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.localization.InitializePinpoint;
import org.firstinspires.ftc.teamcode.turret.HiveOpeningAim.Alliance;
import org.firstinspires.ftc.teamcode.turret.HiveOpeningAim.CameraMount;
import org.firstinspires.ftc.teamcode.turret.HiveOpeningAim.OpeningAim;
import org.firstinspires.ftc.teamcode.turret.HiveOpeningAim.Vector;

import java.util.List;
import java.util.Locale;

/**
 * Aims POLLEN at the upward BIOBUZZ CELL and provides both launcher bearings.
 *
 * Standalone subsystem, NOT an OpMode. Call start(), update(), addTelemetry(),
 * and stop() from the eventual TeleOp.
 *
 * When the direct aiming angle is beyond the mechanical limits, this class
 * tries an equivalent angle +/- 360 degrees. That alternative is possible
 * ONLY when the turret's actual geared travel includes that angle.
 * A positional servo cannot rotate through its mechanical end stops.
 *
 * No turret encoder is used: all turret angles are estimates from servo commands.
 * When optional odometry is supplied, the latest detected opening is anchored
 * in the robot odometry frame and used while the tag is out of view.
 * Note: a moving HIVE can invalidate a remembered opening. Clear on a flip.
 * Set the external gearing ratio, direction and home alignment correctly.
 */
public final class PollenTurret {

    public enum TargetCell {
        ANY_CELL,
        AUDIENCE_SIDE,
        OPPOSITE_AUDIENCE_SIDE,
        /** @deprecated Use ANY_CELL. */
        @Deprecated ANY_BLUE
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
    // With odometry: pivot from the Pinpoint robot reference, forward/left/up meters.
    private static final Vector POLLEN_AXIS_ORIGIN = new Vector(0, 0, 0);

    private static final double TRACKING_KP = 0.45;
    private static final double AIM_TOLERANCE_DEGREES = 1.25;
    private static final double MAX_TURRET_CORRECTION_DEGREES = 10.0;
    private static final double MAX_SERVO_SPEED_DEGREES_PER_SEC = 240.0;
    private static final double TAG_LOST_TIMEOUT_SECONDS = 0.40;
    // The HIVE can flip. Do not trust a remembered opening indefinitely.
    private static final double DEFAULT_ODOMETRY_TARGET_MAX_AGE_SECONDS = 10.0;
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
    private final Alliance alliance;
    private final InitializePinpoint pinpoint; // null keeps the original AprilTag-only behavior
    private final OdometryTargetTracker targetTracker = new OdometryTargetTracker();
    private OdometryTargetTracker.Pose latestOdometryPose;
    private double odometryTargetMaxAgeSeconds = DEFAULT_ODOMETRY_TARGET_MAX_AGE_SECONDS;
    private boolean odometryAiming = false;
    private long odometryFrameId;

    private CameraMount cameraMount;
    private Vector pollenAxis;
    private Vector nectarAxis;
    private OpeningAim openingAim;
    private long servoSettledAtNanos;
    private TargetCell targetCell = TargetCell.ANY_CELL;
    private boolean started = false;
    private double commandedServoPosition;
    private double targetServoPosition;
    private double lastFrameTimestamp = Double.NaN;
    private long lastUpdateNanos = 0L;
    private long lastAcceptedTagNanos = 0L;
    private int trackedTagId = -1;
    private double txDegrees = Double.NaN;
    private String trackingStatus = "NOT STARTED";

    private boolean wrapping = false;
    private long wrapStartedNanos = 0L;
    private long wrapArrivedNanos = 0L;
    private long wrapFailedNanos = 0L;
    private int wrapTagId = -1;
    private double wrapTargetAngleDegrees = Double.NaN;

    /** AprilTag-only aiming with the default pipeline and calibration. */
    public PollenTurret(HardwareMap hardwareMap) {
        this(hardwareMap, DEFAULT_APRILTAG_PIPELINE, DEFAULT_CALIBRATION, null);
    }

    /** Pass the same InitializePinpoint instance as FieldCentricDrive, or null.
     * The OpMode updates it once before updating drive and turret. */
    public PollenTurret(HardwareMap hardwareMap, InitializePinpoint pinpoint) {
        this(hardwareMap, DEFAULT_APRILTAG_PIPELINE, DEFAULT_CALIBRATION, pinpoint);
    }

    /** Choose a pipeline with optional Pinpoint and the default servo calibration. */
    public PollenTurret(HardwareMap hardwareMap, int aprilTagPipeline,
                        InitializePinpoint pinpoint) {
        this(hardwareMap, aprilTagPipeline, DEFAULT_CALIBRATION, pinpoint);
    }

    /** All setup is performed here; null Pinpoint selects AprilTag-only aiming. */
    public PollenTurret(HardwareMap hardwareMap, int aprilTagPipeline,
                        TurretServo.Calibration calibration, InitializePinpoint pinpoint) {
        this(hardwareMap, aprilTagPipeline, calibration, pinpoint, Alliance.BLUE);
    }

    /** Select the alliance with the default pipeline and calibration. Pinpoint may be null. */
    public PollenTurret(HardwareMap hardwareMap, InitializePinpoint pinpoint, Alliance alliance) {
        this(hardwareMap, DEFAULT_APRILTAG_PIPELINE, DEFAULT_CALIBRATION, pinpoint, alliance);
    }

    public PollenTurret(HardwareMap hardwareMap, int aprilTagPipeline,
                        TurretServo.Calibration calibration, InitializePinpoint pinpoint,
                        Alliance alliance) {
        if (alliance == null) {
            throw new IllegalArgumentException("Select a turret alliance.");
        }
        this.alliance = alliance;
        if (aprilTagPipeline < 0 || calibration == null) {
            throw new IllegalArgumentException("Set a valid pipeline and POLLEN calibration.");
        }
        this.aprilTagPipeline = aprilTagPipeline;
        this.calibration = calibration;
        this.pinpoint = pinpoint;
        commandedServoPosition = calibration.home;
        targetServoPosition = calibration.home;
        servo = new TurretServo(hardwareMap, SERVO_NAME);
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        if (hasDefaultCameraMeasurements()) {
            // POLLEN-only aiming uses the POLLEN pivot as the origin for both bearings.
            configureOpeningAim(DEFAULT_CAMERA_MOUNT, POLLEN_AXIS_ORIGIN, POLLEN_AXIS_ORIGIN);
        }
    }

    /** Set the POLLEN pivot relative to the localizer robot origin, in meters. */
    public void configurePollenAxis(Vector axis) {
        configureOpeningAim(cameraMount, axis, axis);
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
        servoSettledAtNanos = lastUpdateNanos + HOME_SETTLE_MS * 1_000_000L;
        clearTarget();
        targetTracker.clear();
        latestOdometryPose = null;
        odometryFrameId = pinpoint == null ? 0 : pinpoint.getPoseFrameId();
        odometryAiming = false;
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
        double elapsedSeconds = Math.min(0.10, Math.max(0.0,
                (now - lastUpdateNanos) / 1_000_000_000.0));
        lastUpdateNanos = now;

        // TeleOp updates the shared odometry device BEFORE this method. We only
        // read its latest pose, so two subsystems never call pinpoint.update().
        long currentFrameId = pinpoint == null ? 0 : pinpoint.getPoseFrameId();
        if (currentFrameId != odometryFrameId) {
            clearOdometryTarget();
            odometryFrameId = currentFrameId;
        }
        latestOdometryPose = readOdometryPose();
        if (latestOdometryPose != null) targetTracker.addPose(now, latestOdometryPose);

        LLResult result = limelight.getLatestResult();
        boolean differentPipelineActive = result != null && result.getPipelineIndex() != aprilTagPipeline;
        boolean acceptedTagFrame = !differentPipelineActive && processFrame(result, now);

        // Also operates if another subsystem temporarily selects a non-tag pipeline.
        odometryAiming = false;
        if (!acceptedTagFrame && pinpoint != null) {
            OpeningAim predicted = targetTracker.fallback(latestOdometryPose,
                    pollenAxis, nectarAxis, now, odometryTargetMaxAgeSeconds);
            if (predicted != null) {
                odometryAiming = true;
                if (!wrapping) {
                    aimAtOpening(predicted, now, false);
                    if (differentPipelineActive) trackingStatus = "ODOMETRY (OTHER PIPELINE)";
                } else {
                    // Keep the planned wrap route, but report updated robot-relative
                    // bearings from odometry even while the tag is still unseen.
                    openingAim = predicted;
                    trackedTagId = predicted.tagId;
                }
            }
        }
        if (differentPipelineActive && !odometryAiming) {
            resetWrap();
            targetServoPosition = commandedServoPosition;
            clearTarget();
            trackingStatus = "OTHER PIPELINE ACTIVE";
        } else {
            updateTrackingTimeouts(now);
        }
        moveServo(now, elapsedSeconds);
    }

    private OdometryTargetTracker.Pose readOdometryPose() {
        if (pinpoint == null) return null;
        Pose2D robotPose = pinpoint.getPose();
        if (robotPose == null) return null;
        OdometryTargetTracker.Pose pose = new OdometryTargetTracker.Pose(
                robotPose.getX(DistanceUnit.METER),
                robotPose.getY(DistanceUnit.METER),
                robotPose.getHeading(AngleUnit.RADIANS));
        return pose.valid() ? pose : null;
    }

    private boolean processFrame(LLResult result, long now) {
        long frameAgeMs = result == null ? -1 : result.getStaleness();
        boolean freshFrame = frameAgeMs >= 0 && frameAgeMs <= MAX_FRAME_STALENESS_MS
                && Double.isFinite(result.getTimestamp())
                && Double.compare(result.getTimestamp(), lastFrameTimestamp) != 0;

        if (freshFrame) {
            lastFrameTimestamp = result.getTimestamp();
            double delayMs = frameAgeMs + result.getCaptureLatency() + result.getTargetingLatency();
            // The camera moves with POLLEN. Use only exposures taken after the last
            // command settled, so its commanded yaw is also the yaw at capture.
            if (Double.isFinite(delayMs) && delayMs >= 0) {
                long captureTime = now - (long) (delayMs * 1_000_000.0);
                if (captureTime >= servoSettledAtNanos) {
                    OdometryTargetTracker.Pose capturePose = pinpoint == null
                            ? null : targetTracker.poseAt(captureTime);
                    if (result.isValid() && aimFromTags(
                            result.getFiducialResults(), now, capturePose)) return true;
                    openingAim = null;
                    if (!wrapping) trackingStatus = "NO STABLE CELL OPENING";
                }
            }
        }
        return false;
    }

    private void updateTrackingTimeouts(long now) {
        if (wrapping) {
            // Never return home merely because the tag is out of the camera's
            // view during the chosen wrap-around route.
            if (Math.abs(commandedServoPosition - targetServoPosition)
                    <= WRAP_ARRIVAL_SERVO_TOLERANCE && wrapArrivedNanos == 0L) {
                wrapArrivedNanos = now;
            }
            // With a valid memorized field target, re-aim using the new odometry
            // position on the next loop instead of requiring tag reacquisition.
            if (wrapArrivedNanos != 0L && pinpoint != null
                    && latestOdometryPose != null
                    && targetTracker.hasTarget(now, odometryTargetMaxAgeSeconds)) {
                resetWrap();
                trackingStatus = "WRAP COMPLETE - ODOMETRY";
                return;
            }

            boolean timedOut = now - wrapStartedNanos >= secondsToNanos(WRAP_MAX_SECONDS);
            boolean failedToReacquire = wrapArrivedNanos != 0L &&
                    now - wrapArrivedNanos >= secondsToNanos(WRAP_REACQUIRE_WAIT_SECONDS);
            if (timedOut || failedToReacquire) {
                resetWrap();
                clearTarget();
                targetTracker.clearTarget(); // avoid repeating an impossible blind wrap
                odometryAiming = false;
                targetServoPosition = calibration.home;
                trackingStatus = "WRAP FAILED - RETURNING HOME";
                wrapFailedNanos = now;
            } else {
                trackingStatus = wrapArrivedNanos != 0L
                        ? "WRAPPED - REACQUIRING TAG" : "WRAPPING INSIDE LIMITS";
            }
        } else {
            if (!odometryAiming && !hasRecentTarget(now)
                    && now >= servoSettledAtNanos + MAX_FRAME_STALENESS_MS * 1_000_000L) {
                clearTarget();
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
    }

    private void moveServo(long now, double elapsedSeconds) {
        double maxStep = MAX_SERVO_SPEED_DEGREES_PER_SEC * elapsedSeconds / calibration.travelDegrees;
        double nextPosition = TurretServo.moveTowards(commandedServoPosition,
                calibration.limit(targetServoPosition), maxStep);
        if (nextPosition != commandedServoPosition) {
            commandedServoPosition = nextPosition;
            servo.setPosition(commandedServoPosition);
            servoSettledAtNanos = now + POLLEN_SETTLE_MS * 1_000_000L;
        }
    }

    private void aimAtOpening(OpeningAim aim, long now, boolean fromAprilTag) {
        trackedTagId = aim.tagId;
        openingAim = aim;
        if (fromAprilTag) lastAcceptedTagNanos = now;

        double currentTurretAngle = calibration.rightDegrees(commandedServoPosition);
        double errorDegrees = openingErrorDegrees(aim.pollenYaw, currentTurretAngle);
        if (Math.abs(errorDegrees) <= AIM_TOLERANCE_DEGREES) {
            targetServoPosition = commandedServoPosition;
            trackingStatus = fromAprilTag ? "ALIGNED" : "ODOMETRY ALIGNED";
            return;
        }

        double desiredTurretAngle = currentTurretAngle + errorDegrees;
        double minAngle = calibration.minRightDegrees;
        double maxAngle = calibration.maxRightDegrees;

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
                trackingStatus = fromAprilTag
                        ? "WRAPPING INSIDE LIMITS" : "ODOMETRY WRAPPING INSIDE LIMITS";
                return;
            }

            // Neither the direct angle nor a 360-degree equivalent is reachable.
            // Move toward the nearest possible boundary without exceeding it.
            double safeAngle = clip(desiredTurretAngle, minAngle, maxAngle);
            setCorrectionTarget(currentTurretAngle, safeAngle - currentTurretAngle);
            trackingStatus = fromAprilTag ? "TARGET OUT OF REACH"
                    : "ODOMETRY TARGET OUT OF REACH";
            return;
        }

        setCorrectionTarget(currentTurretAngle, errorDegrees);
        trackingStatus = fromAprilTag ? "TRACKING" : "ODOMETRY FALLBACK";
    }

    private void setCorrectionTarget(double currentRightDegrees, double errorDegrees) {
        double correction = clip(TRACKING_KP * errorDegrees,
                -MAX_TURRET_CORRECTION_DEGREES, MAX_TURRET_CORRECTION_DEGREES);
        targetServoPosition = calibration.limit(calibration.position(currentRightDegrees + correction));
    }

    static double openingErrorDegrees(double openingYawRadians, double currentRightDegrees) {
        // The geometry reports positive left. The servo mapping uses positive right.
        return Math.IEEEremainder(-Math.toDegrees(openingYawRadians) - currentRightDegrees,
                FULL_TURN_DEGREES);
    }

    /** Optional: restrict tracking to one of the two cells for the selected alliance. */
    public void setTargetCell(TargetCell cell) {
        if (cell == null) {
            throw new IllegalArgumentException("Target cell cannot be null.");
        }
        targetCell = cell;
        resetWrap();
        targetServoPosition = commandedServoPosition;
        clearTarget();
        targetTracker.clear();
        odometryAiming = false;
    }

    /** Explicitly invalidate the cached point when the HIVE flips or pose resets. */
    public void clearOdometryTarget() {
        targetTracker.clear();
        odometryAiming = false;
        clearTarget();
        resetWrap();
        targetServoPosition = calibration.home;
    }

    /** How long a stored opening can be used without another valid AprilTag. */
    public void setOdometryTargetMaxAgeSeconds(double seconds) {
        if (!Double.isFinite(seconds) || seconds <= 0)
            throw new IllegalArgumentException("Odometry cache age must be positive and finite.");
        odometryTargetMaxAgeSeconds = seconds;
    }

    public Vector getRememberedFieldOpening() {
        return targetTracker.hasTarget(System.nanoTime(), odometryTargetMaxAgeSeconds)
                ? targetTracker.getFieldOpening() : null;
    }

    public boolean isOdometryAiming() { return odometryAiming; }

    public boolean hasRecentBlueTag() {
        return alliance == Alliance.BLUE && hasRecentAllianceTag();
    }

    public boolean hasRecentAllianceTag() {
        return started && hasRecentTarget(System.nanoTime());
    }

    public int getTrackedTagId() {
        return trackedTagId;
    }

    /** Latest usable target and both bearings, with positive yaw pointing left. */
    public OpeningAim getOpeningAim() {
        return (hasRecentAllianceTag() || odometryAiming) ? openingAim : null;
    }

    public void addInitializationTelemetry(Telemetry telemetry) {
        telemetry.addLine("AprilTag turret: aims at the upward CELL opening.");
        telemetry.addLine("Check camera mount, turret axes, servo calibration, and 3D pipeline.");
        addTelemetry(telemetry);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret status", trackingStatus);
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Target cell", targetCell);
        telemetry.addData("AprilTag pipeline", aprilTagPipeline);
        telemetry.addData("Odometry enabled", pinpoint != null);
        telemetry.addData("Odometry aim active", odometryAiming);
        telemetry.addData("Odometry pose valid", latestOdometryPose != null);
        if (pinpoint != null && latestOdometryPose != null) {
            telemetry.addData("Odometry robot X/Y (m)", "%.3f / %.3f",
                    latestOdometryPose.x, latestOdometryPose.y);
            telemetry.addData("Odometry heading (deg)", "%.1f",
                    Math.toDegrees(latestOdometryPose.heading));
        }
        Vector remembered = getRememberedFieldOpening();
        if (remembered != null) {
            telemetry.addData("Remembered CELL field X/Y/Z (m)", "%.3f / %.3f / %.3f",
                    remembered.x, remembered.y, remembered.z);
            telemetry.addData("Remembered CELL age (s)", "%.2f",
                    targetTracker.ageSeconds(System.nanoTime()));
        }
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
        targetTracker.clear();
        odometryAiming = false;
        resetWrap();
        trackingStatus = "STOPPED";
        limelight.stop();
    }

    private boolean hasRecentTarget(long now) {
        return lastAcceptedTagNanos != 0L && now - lastAcceptedTagNanos < secondsToNanos(TAG_LOST_TIMEOUT_SECONDS);
    }

    private void clearTarget() {
        lastAcceptedTagNanos = 0L;
        openingAim = null;
        trackedTagId = -1;
        txDegrees = Double.NaN;
    }

    private void resetWrap() {
        wrapping = false;
        wrapStartedNanos = 0L;
        wrapArrivedNanos = 0L;
        wrapTagId = -1;
        wrapTargetAngleDegrees = Double.NaN;
    }

    private boolean aimFromTags(List<LLResultTypes.FiducialResult> tags, long now,
                                OdometryTargetTracker.Pose capturePose) {
        if (tags == null || (wrapping && wrapArrivedNanos == 0L)) return false;
        double pollenYaw = Math.toRadians(-calibration.rightDegrees(commandedServoPosition));
        OpeningAim best = null;
        LLResultTypes.FiducialResult bestTag = null;
        for (LLResultTypes.FiducialResult tag : tags) {
            if (tag == null || !isAllowedTag(tag.getFiducialId())) continue;
            if (wrapping && tag.getFiducialId() != wrapTagId) continue;
            OpeningAim aim = HiveOpeningAim.estimateOpening(alliance,
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
        if (pinpoint != null && latestOdometryPose != null && capturePose == null) return false;
        if (wrapping) resetWrap();
        txDegrees = bestTag.getTargetXDegrees();
        // Cache an observation in the odometry field frame *at image exposure*.
        // With no pinpoint, tracking behaves exactly as before.
        if (pinpoint != null && capturePose != null) {
            targetTracker.observe(best, capturePose, now);
            // Aim from the current robot pose, not the older camera exposure pose.
            OpeningAim currentAim = targetTracker.fallback(latestOdometryPose,
                    pollenAxis, nectarAxis, now, odometryTargetMaxAgeSeconds);
            if (currentAim != null) best = currentAim;
        }
        aimAtOpening(best, now, true);
        return true;
    }

    private boolean isAllowedTag(int id) {
        if (!alliance.accepts(id)) return false;
        switch (targetCell) {
            case AUDIENCE_SIDE:
                return alliance.isAudienceTag(id);
            case OPPOSITE_AUDIENCE_SIDE:
                return !alliance.isAudienceTag(id);
            case ANY_CELL:
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
