package org.firstinspires.ftc.teamcode.turret;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * One SPEED servo drives two mechanically synchronized, parallel turrets.
 * The midpoint between their separate axes aims at the blue BIOBUZZ hive pivot.
 * Position mode lets the servo control its shaft angle using internal feedback.
 * Turret telemetry reports COMMANDS, not externally measured turret angles.
 * No drivetrain hardware is created or commanded by this class.
 * See docs/BLUE_HIVE_TURRET.md for calibration and OpMode integration.
 */
public final class BlueHiveTurret {
    /** Edit this method once; every integrated OpMode uses these settings. */
    public static Settings defaultSettings() {
        return new Settings();
    }

    public static final class Settings {
        public String servoName = "servo";
        // 2000-0025-0003 (SPEED) must remain in its original POSITION mode.
        public boolean motionEnabled = false;
        // External gearing only: servo output turns / turret turn. NOT internal 135:1.
        public double servoTurnsPerTurretTurn = Double.NaN;
        // Both turret gear trains MUST have the same ratio and rotation direction.
        // +1 or -1: sign of increasing Servo position relative to turret CCW.
        public double increasingPositionCcwSign = Double.NaN;
        // CALIBRATE: Servo position (0..1) with BOTH turrets pointing robot-forward.
        public double servoForwardPosition = Double.NaN;
        // Travel across positions 0..1 with the 500-2500 us PWM range below.
        public double servoTravelDegrees = 300.0;
        // Reduce these after checking the mechanism's safe shaft endpoints.
        public double servoMinimumPosition = 0.0;
        public double servoMaximumPosition = 1.0;

        // Inches. Origin: bottom-left INSIDE corner of the supplied field diagram.
        // +X towards blue alliance wall, +Y towards the top; heading 0 = +X, CCW positive.
        // Nominal 144-inch field; pivots 25.5 inches apart, symmetric about its center.
        // FIRST manual Fig. 9-10: https://ftc-resources.firstinspires.org/ftc/archive/2027/game/manual-09
        public double hiveX = 84.75;
        public double hiveY = 72.0;
        // The pair's MIDPOINT, not either individual turret axis, relative to Pedro.
        public double pairCenterForwardInches = -5.0;
        public double pairCenterLeftInches = 0.0;
        public double turretAxisSpacingInches = 7.0;
        public double cableMinimumDegrees = -360.0;
        public double cableMaximumDegrees = 360.0;
        public double cableMarginDegrees = 15.0;
        public double minimumTargetDistanceInches = 1.0;
    }

    private static final long HANDOFF_LIFETIME_NANOS = 300_000_000_000L;
    private static Handoff handoff;
    private final Settings settings;
    private final AimModel model;
    private final Servo servo;
    private final PinpointLocalizer teleOpLocalizer;
    private final boolean autonomous;
    private final String calibrationProblem;
    private boolean running;
    private boolean hasStarted;
    private Pose lastPose;
    private String status = "Initialized; waiting for start";

    /** Call after creating the autonomous follower and setting its starting pose. */
    public static BlueHiveTurret forAutonomous(HardwareMap hardwareMap) {
        clearHandoff();
        return new BlueHiveTurret(hardwareMap, defaultSettings(), null, null, true);
    }

    /**
     * Uses the last integrated autonomous, or a caller-supplied standalone pose.
     * Forward must be calibrated to a fixed servo position with untwisted wires.
     * The handoff is consumed once, expires after 5 minutes, and is lost on app restart.
     */
    public static BlueHiveTurret forTeleOp(HardwareMap hardwareMap, Pose standalonePose) {
        Handoff saved = takeHandoff();
        Pose startingPose = saved == null ? standalonePose : saved.pose;
        if (!validPose(startingPose)) {
            throw new IllegalArgumentException("No autonomous handoff: supply a measured standalone pose.");
        }
        // Copy robot-specific configuration, overriding ONLY reset mode for this new instance.
        // Do not mutate Constants.localizerConfig, recalibrate IMU, or create a drivetrain.
        PinpointConfig source = Constants.localizerConfig;
        PinpointConfig continuation = new PinpointConfig(c -> {
            c.name.set(source.name.get());
            c.xPodDirection.set(source.xPodDirection.get());
            c.yPodDirection.set(source.yPodDirection.get());
            c.xPodOffset.set(source.xPodOffset.get());
            c.yPodOffset.set(source.yPodOffset.get());
            c.offsetUnits.set(source.offsetUnits.get());
            c.globalDistanceUnit.set(source.globalDistanceUnit.get());
            c.encoderResolutionUnit.set(source.encoderResolutionUnit.get());
            c.podType.set(source.podType.get());
            c.ticksPerUnit.set(source.ticksPerUnit.get());
            c.resetMode.set(PinpointLocalizer.ResetMode.NONE);
        });
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, continuation);
        // Robot stays stationary across the transition, as specified by the team.
        localizer.setPose(startingPose);
        return new BlueHiveTurret(hardwareMap, defaultSettings(), localizer, saved, false);
    }

    /** Blue-wall midpoint, facing the hive. Measure wall-to-PEDRO-reference distance. */
    public static Pose blueWallStartPose(double wallToTrackedPointInches) {
        if (!Double.isFinite(wallToTrackedPointInches)
                || wallToTrackedPointInches <= 0.0 || wallToTrackedPointInches >= 59.25) {
            throw new IllegalArgumentException("Measure the wall-to-tracked-point distance (0 < inches < 59.25).");
        }
        return new Pose(144.0 - wallToTrackedPointInches, 72.0, Math.PI);
    }

    public static synchronized void clearHandoff() {
        handoff = null;
    }

    private static synchronized Handoff takeHandoff() {
        Handoff saved = handoff;
        handoff = null;
        if (saved == null || System.nanoTime() - saved.savedNanos > HANDOFF_LIFETIME_NANOS) {
            return null;
        }
        return saved;
    }

    private BlueHiveTurret(HardwareMap hardwareMap, Settings settings,
                           PinpointLocalizer localizer, Handoff saved, boolean autonomous) {
        this.settings = settings;
        this.teleOpLocalizer = localizer;
        this.autonomous = autonomous;
        model = new AimModel(settings, saved == null ? Double.NaN : saved.commandedDegrees);
        if (saved != null) {
            lastPose = saved.pose;
        }
        calibrationProblem = settingsProblem(settings);
        if (settings.motionEnabled && calibrationProblem == null) {
            servo = hardwareMap.get(Servo.class, settings.servoName);
            // Predictable position mapping; gear reversal is handled by the calibrated sign.
            servo.setDirection(Servo.Direction.FORWARD);
            if (!(servo instanceof PwmControl)) {
                throw new IllegalStateException("Position mapping requires configurable 500-2500 us PWM.");
            }
            ((PwmControl) servo).setPwmRange(new PwmControl.PwmRange(500, 2500));
            // Do not command forward during init or the Auto-to-TeleOp transition.
        } else {
            servo = null;
            status = calibrationProblem == null ? "Disabled: motionEnabled=false" : calibrationProblem;
        }
    }

    /** Call at OpMode start, NOT init. The first valid update sends a position target. */
    public void start() {
        running = true;
        hasStarted = true;
    }

    /** Autonomous: call after follower.update(). Never updates or commands that follower. */
    public void update(Pose robotPose) {
        lastPose = validPose(robotPose) ? robotPose : null;
        model.plan(robotPose);
        if (calibrationProblem != null) {
            status = calibrationProblem;
        } else if (!settings.motionEnabled) {
            status = "Disabled: motionEnabled=false";
        } else if (!running) {
            status = "Waiting for start / stopped";
        } else if (!model.targetAvailable) {
            status = model.bearingAvailable ? "Hive outside position range; keeping previous target"
                    : "No valid pose or target too close; keeping previous target";
        } else {
            servo.setPosition(model.targetPosition);
            model.acceptTarget();
            status = "Position target sent; actual turret angle is not measured";
        }
        if (autonomous && running) {
            saveHandoff();
        }
    }

    /** TeleOp: update Pinpoint and aiming. Your drive code keeps sole control of drive motors. */
    public Pose updateLocalizationAndAim() {
        if (teleOpLocalizer == null) {
            throw new IllegalStateException("Autonomous must pass follower.pose() to update().");
        }
        try {
            teleOpLocalizer.update();
            Pose pose = teleOpLocalizer.pose();
            update(pose);
            return pose;
        } catch (RuntimeException error) {
            stop();
            throw error;
        }
    }

    /** Stop sending targets. Position mode retains the last target until SDK PWM shutdown. */
    public void stop() {
        running = false;
        status = "Updates stopped; last position target retained";
        // setPosition(0) would move to an endpoint, not stop. Do not return forward.
        if (autonomous && hasStarted) {
            saveHandoff();
        }
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret status", status);
        telemetry.addData("Parallel turret angle COMMANDED", "%.1f deg", model.commandedDegrees);
        telemetry.addData("Pair midpoint target", "%.1f deg", model.targetDegrees);
        telemetry.addData("Requested hive bearing", "%.1f deg", model.requestedDegrees);
        telemetry.addData("Servo position target", "%.3f", model.targetPosition);
        telemetry.addData("Reachable turret range", "%.1f to %.1f deg", model.minimumDegrees, model.maximumDegrees);
        telemetry.addData("Turret axis spacing", "%.1f in", settings.turretAxisSpacingInches);
        telemetry.addData("Pair midpoint X/Y", "%.1f / %.1f in", model.pairX, model.pairY);
        telemetry.addData("Turret angle feedback", "Servo internal; no measured angle returned to app");
    }

    private void saveHandoff() {
        synchronized (BlueHiveTurret.class) {
            handoff = validPose(lastPose) ? new Handoff(lastPose, model) : null;
        }
    }

    private static final class Handoff {
        final Pose pose;
        final double commandedDegrees;
        final long savedNanos = System.nanoTime();

        Handoff(Pose pose, AimModel model) {
            this.pose = new Pose(pose.x(), pose.y(), pose.heading());
            commandedDegrees = model.commandedDegrees;
        }
    }

    static boolean validPose(Pose pose) {
        return pose != null && Double.isFinite(pose.x()) && Double.isFinite(pose.y())
                && Double.isFinite(pose.heading());
    }

    private static String settingsProblem(Settings s) {
        if (!positive(s.servoTurnsPerTurretTurn)) return "Disabled: external gear ratio not calibrated";
        if (s.increasingPositionCcwSign != 1.0 && s.increasingPositionCcwSign != -1.0)
            return "Disabled: verify increasing-position direction (+1 or -1)";
        if (!positive(s.servoTravelDegrees)) return "Disabled: invalid servo travel";
        if (!Double.isFinite(s.servoMinimumPosition) || !Double.isFinite(s.servoMaximumPosition)
                || s.servoMinimumPosition < 0 || s.servoMaximumPosition > 1
                || s.servoMinimumPosition >= s.servoMaximumPosition)
            return "Disabled: invalid safe servo endpoints";
        if (!Double.isFinite(s.servoForwardPosition) || s.servoForwardPosition < s.servoMinimumPosition
                || s.servoForwardPosition > s.servoMaximumPosition)
            return "Disabled: calibrate forward within safe servo endpoints";
        return null;
    }

    private static boolean positive(double value) {
        return Double.isFinite(value) && value > 0.0;
    }

    /** Hardware-independent geometry, reachable-angle selection and position mapping. */
    static final class AimModel {
        final Settings settings;
        double commandedDegrees;
        double requestedDegrees = Double.NaN;
        double targetDegrees = Double.NaN;
        double targetPosition = Double.NaN;
        double minimumDegrees = Double.NaN;
        double maximumDegrees = Double.NaN;
        double pairX = Double.NaN;
        double pairY = Double.NaN;
        double leftAxisX = Double.NaN;
        double leftAxisY = Double.NaN;
        double rightAxisX = Double.NaN;
        double rightAxisY = Double.NaN;
        boolean bearingAvailable;
        boolean targetAvailable;

        AimModel(Settings settings, double startingDegrees) {
            this.settings = settings;
            commandedDegrees = startingDegrees;
            if (!Double.isFinite(settings.hiveX) || !Double.isFinite(settings.hiveY)
                    || !Double.isFinite(settings.pairCenterForwardInches) || !Double.isFinite(settings.pairCenterLeftInches)
                    || !positive(settings.turretAxisSpacingInches)
                    || !Double.isFinite(settings.cableMinimumDegrees) || !Double.isFinite(settings.cableMaximumDegrees)
                    || !Double.isFinite(settings.cableMarginDegrees) || settings.cableMarginDegrees < 0
                    || settings.cableMinimumDegrees + settings.cableMarginDegrees > 0.0
                    || settings.cableMaximumDegrees - settings.cableMarginDegrees < 0.0
                    || settings.cableMinimumDegrees + settings.cableMarginDegrees
                    >= settings.cableMaximumDegrees - settings.cableMarginDegrees
                    || !positive(settings.minimumTargetDistanceInches)) {
                throw new IllegalArgumentException("Invalid geometry/cable limits; untwisted forward must be allowed.");
            }
        }

        void plan(Pose pose) {
            targetAvailable = false;
            bearingAvailable = false;
            requestedDegrees = targetDegrees = targetPosition = minimumDegrees = maximumDegrees = Double.NaN;
            pairX = pairY = leftAxisX = leftAxisY = rightAxisX = rightAxisY = Double.NaN;
            if (!validPose(pose)) return;
            double cosine = Math.cos(pose.heading());
            double sine = Math.sin(pose.heading());
            pairX = pose.x() + settings.pairCenterForwardInches * cosine - settings.pairCenterLeftInches * sine;
            pairY = pose.y() + settings.pairCenterForwardInches * sine + settings.pairCenterLeftInches * cosine;
            double halfSpacing = settings.turretAxisSpacingInches / 2.0;
            leftAxisX = pairX - halfSpacing * sine;
            leftAxisY = pairY + halfSpacing * cosine;
            rightAxisX = pairX + halfSpacing * sine;
            rightAxisY = pairY - halfSpacing * cosine;
            // Parallel barrels share ONE bearing, from the midpoint. Do not converge them.
            double dx = settings.hiveX - pairX;
            double dy = settings.hiveY - pairY;
            if (Math.hypot(dx, dy) < settings.minimumTargetDistanceInches) return;
            double bearing = Math.toDegrees(Math.atan2(dy, dx) - pose.heading());
            requestedDegrees = ((bearing % 360.0) + 540.0) % 360.0 - 180.0;
            bearingAvailable = Double.isFinite(requestedDegrees);
            if (!bearingAvailable || settingsProblem(settings) != null) return;
            double endpointA = angleForPosition(settings.servoMinimumPosition);
            double endpointB = angleForPosition(settings.servoMaximumPosition);
            minimumDegrees = Math.max(Math.min(endpointA, endpointB),
                    settings.cableMinimumDegrees + settings.cableMarginDegrees);
            maximumDegrees = Math.min(Math.max(endpointA, endpointB),
                    settings.cableMaximumDegrees - settings.cableMarginDegrees);
            double reference = Double.isFinite(commandedDegrees) ? commandedDegrees : 0.0;
            targetDegrees = nearestEquivalent(requestedDegrees, reference);
            if (!Double.isFinite(targetDegrees)) return; // Do not clamp an unreachable bearing into a false aim.
            targetPosition = positionForAngle(targetDegrees);
            targetAvailable = Double.isFinite(targetPosition)
                    && targetPosition >= settings.servoMinimumPosition - 1e-9
                    && targetPosition <= settings.servoMaximumPosition + 1e-9;
            if (targetAvailable) targetPosition = Math.max(settings.servoMinimumPosition,
                    Math.min(settings.servoMaximumPosition, targetPosition));
        }

        void acceptTarget() {
            if (targetAvailable) commandedDegrees = targetDegrees;
        }

        double positionForAngle(double angle) {
            return settings.servoForwardPosition + settings.increasingPositionCcwSign
                    * angle * settings.servoTurnsPerTurretTurn / settings.servoTravelDegrees;
        }

        double angleForPosition(double position) {
            return (position - settings.servoForwardPosition) * settings.servoTravelDegrees
                    / (settings.increasingPositionCcwSign * settings.servoTurnsPerTurretTurn);
        }

        private double nearestEquivalent(double degrees, double reference) {
            double firstTurn = Math.ceil((minimumDegrees - degrees - 1e-9) / 360.0);
            double lastTurn = Math.floor((maximumDegrees - degrees + 1e-9) / 360.0);
            double turn = Math.max(firstTurn, Math.min(lastTurn, Math.rint((reference - degrees) / 360.0)));
            return firstTurn > lastTurn ? Double.NaN : degrees + 360.0 * turn;
        }
    }
}
