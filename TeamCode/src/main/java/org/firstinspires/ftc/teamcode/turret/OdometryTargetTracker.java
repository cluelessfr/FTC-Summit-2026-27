package org.firstinspires.ftc.teamcode.turret;

import java.util.ArrayDeque;
import java.util.Iterator;

import org.firstinspires.ftc.teamcode.turret.HiveOpeningAim.Vector;
import org.firstinspires.ftc.teamcode.turret.HiveOpeningAim.OpeningAim;

/**
 * Keeps the last observed CELL opening fixed in the *local odometry field frame*.
 * A Pose is a field position and heading of the ROBOT origin (not the turret pivot).
 * History permits approximate compensation for Limelight image latency.
 * No fixed field landmarks or assumed absolute origin are needed.
 */
final class OdometryTargetTracker {
    static final class Pose {
        final double x, y, heading;
        Pose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
        boolean valid() {
            return Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(heading);
        }
    }

    private static final class TimedPose {
        final long sampledAtNanos;
        final Pose pose;
        TimedPose(long sampledAtNanos, Pose pose) {
            this.sampledAtNanos = sampledAtNanos;
            this.pose = pose;
        }
    }

    private static final long POSE_HISTORY_NANOS = 2_000_000_000L;
    private static final long POSE_ENDPOINT_TOLERANCE_NANOS = 40_000_000L;
    private static final int MAX_POSES = 128;
    private final ArrayDeque<TimedPose> history = new ArrayDeque<>();
    private Vector fieldOpening;
    private long observedAtNanos;
    private int tagId = -1;
    private boolean audienceCellUp;

    void clear() {
        clearTarget();
        history.clear();
    }

    void clearTarget() {
        fieldOpening = null;
        observedAtNanos = 0;
        tagId = -1;
    }

    void addPose(long now, Pose pose) {
        if (pose == null || !pose.valid()) return;
        if (!history.isEmpty() && now <= history.peekLast().sampledAtNanos) return;
        history.addLast(new TimedPose(now, pose));
        while (!history.isEmpty() &&
                (now - history.peekFirst().sampledAtNanos > POSE_HISTORY_NANOS || history.size() > MAX_POSES)) {
            history.removeFirst();
        }
    }

    Pose poseAt(long captureNanos) {
        if (history.isEmpty()) return null;
        TimedPose first = history.peekFirst();
        TimedPose last = history.peekLast();
        if (captureNanos < first.sampledAtNanos - POSE_ENDPOINT_TOLERANCE_NANOS
                || captureNanos > last.sampledAtNanos + POSE_ENDPOINT_TOLERANCE_NANOS) return null;
        if (captureNanos <= first.sampledAtNanos) return first.pose;
        if (captureNanos >= last.sampledAtNanos) return last.pose;

        Iterator<TimedPose> samples = history.iterator();
        TimedPose before = samples.next();
        while (samples.hasNext()) {
            TimedPose after = samples.next();
            if (captureNanos <= after.sampledAtNanos) {
                double fraction = (captureNanos - before.sampledAtNanos) / (double) (after.sampledAtNanos - before.sampledAtNanos);
                double headingDelta = Math.IEEEremainder(after.pose.heading - before.pose.heading, 2 * Math.PI);
                return new Pose(before.pose.x + fraction * (after.pose.x - before.pose.x),
                        before.pose.y + fraction * (after.pose.y - before.pose.y),
                        before.pose.heading + fraction * headingDelta);
            }
            before = after;
        }
        return null;
    }

    void observe(OpeningAim aim, Pose poseAtCapture, long now) {
        if (aim == null || poseAtCapture == null || !poseAtCapture.valid()) return;
        // Robot-relative -> field (XY planar rotation). Keep physical Z in meters.
        fieldOpening = robotToField(aim.opening, poseAtCapture);
        tagId = aim.tagId;
        audienceCellUp = aim.audienceCellUp;
        observedAtNanos = now;
    }

    boolean hasTarget(long now, double maxAgeSeconds) {
        return fieldOpening != null && observedAtNanos != 0
                && now >= observedAtNanos
                && (now - observedAtNanos) <= maxAgeSeconds * 1e9;
    }

    OpeningAim fallback(Pose currentPose, Vector pollenAxis, Vector nectarAxis,
                        long now, double maxAgeSeconds) {
        if (currentPose == null || !currentPose.valid() || !hasTarget(now, maxAgeSeconds)) return null;
        Vector relative = fieldToRobot(fieldOpening, currentPose);
        return HiveOpeningAim.fromRememberedOpening(tagId, audienceCellUp,
                relative, pollenAxis, nectarAxis);
    }

    Vector getFieldOpening() { return fieldOpening; }
    double ageSeconds(long now) {
        return observedAtNanos == 0 ? Double.NaN : (now - observedAtNanos) / 1e9;
    }

    static Vector robotToField(Vector local, Pose pose) {
        double cosHeading = Math.cos(pose.heading), sinHeading = Math.sin(pose.heading);
        return new Vector(pose.x + cosHeading * local.x - sinHeading * local.y,
                pose.y + sinHeading * local.x + cosHeading * local.y, local.z);
    }

    static Vector fieldToRobot(Vector field, Pose pose) {
        double cosHeading = Math.cos(pose.heading), sinHeading = Math.sin(pose.heading);
        double fieldDeltaX = field.x - pose.x, fieldDeltaY = field.y - pose.y;
        return new Vector(cosHeading * fieldDeltaX + sinHeading * fieldDeltaY,
                -sinHeading * fieldDeltaX + cosHeading * fieldDeltaY, field.z);
    }
}
