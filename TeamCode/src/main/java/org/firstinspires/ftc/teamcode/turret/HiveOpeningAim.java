package org.firstinspires.ftc.teamcode.turret;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/** Computes the upward CELL opening and independent turret bearings from a HIVE tag pose. */
public final class HiveOpeningAim {

    private HiveOpeningAim() { }

    private static final double[] TAG_OFFSET_X_METERS = {-0.165, -0.070, 0.070, 0.165};
    private static final double SAME_CELL_OFFSET_Y_METERS = 0.182;
    private static final double OTHER_CELL_OFFSET_Y_METERS = -0.908;
    private static final double OFFSET_INTO_TAG_METERS = 0.146;
    private static final double AIM_DEPTH_METERS = 0.0508; // 2 inches behind the opening rim.

    public enum Alliance {
        BLUE(38, 38),
        RED(30, 34);

        private final int firstTag;
        private final int firstAudienceTag;

        Alliance(int firstTag, int firstAudienceTag) {
            this.firstTag = firstTag;
            this.firstAudienceTag = firstAudienceTag;
        }

        boolean accepts(int id) {
            return id >= firstTag && id < firstTag + 8;
        }

        boolean isAudienceTag(int id) {
            return id >= firstAudienceTag && id < firstAudienceTag + 4;
        }
    }

    // Robot and turret vectors use forward, left, up in meters.
    public static final class Vector {
        public final double x, y, z;

        public Vector(double x, double y, double z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        Vector plus(Vector v) {
            return new Vector(x + v.x, y + v.y, z + v.z);
        }

        Vector minus(Vector v) {
            return new Vector(x - v.x, y - v.y, z - v.z);
        }

        double dot(Vector v) {
            return x * v.x + y * v.y + z * v.z;
        }

        boolean isFinite() {
            return Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(z);
        }

        // Intrinsic Z-Y-X rotation (yaw, pitch, roll), in radians.
        Vector rotate(double yaw, double pitch, double roll) {
            double cr = Math.cos(roll), sr = Math.sin(roll);
            double cp = Math.cos(pitch), sp = Math.sin(pitch);
            double cy = Math.cos(yaw), sy = Math.sin(yaw);
            double rolledY = cr * y - sr * z;
            double rolledZ = sr * y + cr * z;
            double pitchedX = cp * x + sp * rolledZ;
            return new Vector(cy * pitchedX - sy * rolledY,
                    sy * pitchedX + cy * rolledY, -sp * x + cp * rolledZ);
        }
    }

    public static final class CameraMount {
        final Vector lensFromPollenAxis;
        final double yaw, pitch, roll;

        public CameraMount(Vector lensFromPollenAxis, double yaw, double pitchUp, double roll) {
            this.lensFromPollenAxis = lensFromPollenAxis;
            this.yaw = yaw;
            this.pitch = -pitchUp;
            this.roll = roll;
        }

        Vector directionAtHome(Vector camera) {
            return new Vector(camera.z, -camera.x, -camera.y).rotate(yaw, pitch, roll);
        }
    }

    public static final class OpeningAim {
        public final int tagId;
        public final boolean audienceCellUp;
        public final Vector opening; // Aim point inside the opening, in robot coordinates.
        public final double pollenYaw, nectarYaw;

        private OpeningAim(int tagId, boolean audienceCellUp, Vector opening,
                           Vector pollenAxis, Vector nectarAxis) {
            this.tagId = tagId;
            this.audienceCellUp = audienceCellUp;
            this.opening = opening;
            pollenYaw = bearingFrom(pollenAxis, opening);
            nectarYaw = bearingFrom(nectarAxis, opening);
        }
    }

    /** Reconstructs bearings from the odometry-predicted opening position. */
    static OpeningAim fromRememberedOpening(int tagId, boolean audienceCellUp,
                                             Vector robotRelativeOpening,
                                             Vector pollenAxis, Vector nectarAxis) {
        if (robotRelativeOpening == null || !robotRelativeOpening.isFinite()
                || pollenAxis == null || nectarAxis == null
                || !pollenAxis.isFinite() || !nectarAxis.isFinite()) return null;
        OpeningAim aim = new OpeningAim(tagId, audienceCellUp,
                robotRelativeOpening, pollenAxis, nectarAxis);
        return Double.isFinite(aim.pollenYaw) && Double.isFinite(aim.nectarYaw) ? aim : null;
    }

    /** Uses the FTC SDK camera-space pose without Limelight map POI offsets. */
    public static OpeningAim estimateOpening(Alliance alliance, LLResultTypes.FiducialResult tag,
                                             CameraMount mount, double pollenYawAtCapture,
                                             Vector pollenAxis, Vector nectarAxis) {
        if (tag == null || !alliance.accepts(tag.getFiducialId())) return null;
        Pose3D pose = tag.getTargetPoseCameraSpace();
        if (pose == null || pose.getPosition() == null || pose.getOrientation() == null) return null;
        Position tagPosition = pose.getPosition().toUnit(DistanceUnit.METER);
        YawPitchRollAngles tagOrientation = pose.getOrientation();
        return estimateOpening(alliance, tag.getFiducialId(), new Vector(tagPosition.x, tagPosition.y, tagPosition.z),
                tagOrientation.getYaw(AngleUnit.RADIANS), tagOrientation.getPitch(AngleUnit.RADIANS),
                tagOrientation.getRoll(AngleUnit.RADIANS), mount, pollenYawAtCapture, pollenAxis, nectarAxis);
    }

    static OpeningAim estimateOpening(Alliance alliance, int id, Vector tag,
                                      double yaw, double pitch, double roll,
                                      CameraMount mount, double pollenYawAtCapture,
                                      Vector pollenAxis, Vector nectarAxis) {
        if (!alliance.accepts(id) || !tag.isFinite() || tag.z <= 0) return null;
        // Limelight's native pose uses right/down/into the tag for its rotation.
        // The measured table uses right/down/outward, so -0.146 m becomes +0.146 m.
        Vector down = new Vector(0, 1, 0).rotate(yaw, pitch, roll);
        Vector inward = new Vector(0, 0, 1).rotate(yaw, pitch, roll);
        double inwardUpComponent = mount.directionAtHome(inward).z;
        // A stable CELL's printed-down axis points toward its opening (about +/-30 degrees).
        // Reject intermediate poses and solutions inconsistent with an underside tag.
        double printedDownUpComponent = mount.directionAtHome(down).z;
        if (!(Math.abs(printedDownUpComponent) >= 0.25 && Math.abs(printedDownUpComponent) <= 0.75
                && inwardUpComponent >= 0.60 && inward.dot(tag) > 0)) return null;

        boolean taggedCellUp = printedDownUpComponent > 0;
        // Printed-down points out of this CELL and into the opposite CELL.
        double openingOffsetY = taggedCellUp ? SAME_CELL_OFFSET_Y_METERS - AIM_DEPTH_METERS
                : OTHER_CELL_OFFSET_Y_METERS + AIM_DEPTH_METERS;
        Vector offset = new Vector(TAG_OFFSET_X_METERS[(id - alliance.firstTag) % 4],
                openingOffsetY, OFFSET_INTO_TAG_METERS);
        Vector lensToOpening = tag.plus(offset.rotate(yaw, pitch, roll));
        Vector pivotToOpeningAtHome = mount.lensFromPollenAxis.plus(mount.directionAtHome(lensToOpening));
        Vector opening = pollenAxis.plus(pivotToOpeningAtHome.rotate(pollenYawAtCapture, 0, 0));
        if (!opening.isFinite()) return null;
        return new OpeningAim(id, alliance.isAudienceTag(id) == taggedCellUp,
                opening, pollenAxis, nectarAxis);
    }

    private static double bearingFrom(Vector launcher, Vector target) {
        Vector delta = target.minus(launcher);
        return Math.hypot(delta.x, delta.y) > 1e-6 ? Math.atan2(delta.y, delta.x) : Double.NaN;
    }

}
