package org.firstinspires.ftc.teamcode.turret;

/** Run with scripts/check-blue-hive-turret.ps1. */
public final class BlueHiveTurretChecks {
    private static int checks;

    public static void main(String[] args) {
        aprilTagOpening();
        cellSwitchConfirmation();
        System.out.println("PASS: " + checks + " AprilTag aim and calibration checks");
    }

    private static void aprilTagOpening() {
        HiveOpeningAim.Vector pollen = new HiveOpeningAim.Vector(0, 0.0889, 0);
        HiveOpeningAim.Vector nectar = new HiveOpeningAim.Vector(0, -0.0889, 0);
        HiveOpeningAim.CameraMount mount = new HiveOpeningAim.CameraMount(
                new HiveOpeningAim.Vector(0, 0, 0), 0, 0, 0);
        double sin60 = Math.sqrt(3) / 2;
        double forward = 2 - 0.8572 * sin60 + 0.146 / 2;
        double up = 0.8572 / 2 + 0.146 * sin60;
        // Tag 38 on the lower CELL, with the opening centered between the turrets.
        HiveOpeningAim.OpeningAim low = HiveOpeningAim.estimateOpening(
                HiveOpeningAim.Alliance.BLUE, 38, new HiveOpeningAim.Vector(0.2539, 0, 2), 0, 0, Math.PI / 3,
                mount, 0, pollen, nectar);
        check(low != null && !low.audienceCellUp, "lower audience tag selects opposite CELL");
        close(low.opening.x, forward, "opposite opening forward");
        close(low.opening.y, 0, "opposite opening centered");
        close(low.opening.z, up, "opposite opening height");
        close(low.pollenYaw, -Math.atan2(0.0889, forward), "POLLEN right yaw");
        close(low.nectarYaw, Math.atan2(0.0889, forward), "NECTAR left yaw");
        close(PollenTurret.openingErrorDegrees(low.pollenYaw, 0),
                Math.toDegrees(Math.atan2(0.0889, forward)), "POLLEN controller turns right");
        close(PollenTurret.openingErrorDegrees(Math.toRadians(-179), -179),
                -2, "POLLEN controller takes the short error across 180 degrees");
        TurretServo.Calibration reversedNectar = new TurretServo.Calibration(0.45, 0.2, 0.65, 300, 1, -1);
        close(reversedNectar.position(-30), 0.55, "NECTAR uses its own home and direction");
        close(reversedNectar.rightDegrees(0.55), -30, "NECTAR position maps back to its angle");
        check(Double.isNaN(reversedNectar.nearestReachableRightDegrees(-80, 0)),
                "NECTAR rejects an unreachable target");
        TurretServo.Calibration widePollen = new TurretServo.Calibration(0.5, 0.05, 0.95, 300, 2, 1);
        close(widePollen.nearestReachableRightDegrees(200, 170), 200, "POLLEN preserves nearby route");
        close(widePollen.nearestReachableRightDegrees(200, -170), -160, "wide travel allows equivalent aim");
        HiveOpeningAim.OpeningAim neighbor = HiveOpeningAim.estimateOpening(
                HiveOpeningAim.Alliance.BLUE, 39, new HiveOpeningAim.Vector(0.1589, 0, 2), 0, 0, Math.PI / 3,
                mount, 0, pollen, nectar);
        close(neighbor.opening.y, low.opening.y, "neighboring tag gives same opening");

        // Printed tag axes rotate with the CELL: verify both state and signed offset.
        HiveOpeningAim.OpeningAim high = HiveOpeningAim.estimateOpening(
                HiveOpeningAim.Alliance.BLUE, 38, new HiveOpeningAim.Vector(-0.165, 0, 2), Math.PI, 0, -Math.PI / 3,
                mount, Math.PI / 2, pollen, nectar);
        check(high != null && high.audienceCellUp, "upper audience tag selects its own CELL");
        close(high.opening.x, 0, "90 degree turret yaw rotates target");
        close(high.opening.y, 0.0889 + 2 - 0.1312 * sin60 + 0.146 / 2, "rotated opening left");
        close(high.opening.z, 0.1312 / 2 + 0.146 * sin60, "same opening height");
        check(HiveOpeningAim.estimateOpening(
                HiveOpeningAim.Alliance.BLUE, 38, new HiveOpeningAim.Vector(0, 0, 2), 0, 0, Math.PI / 2,
                mount, 0, pollen, nectar) == null, "neutral CELL produces no aim");
        for (int tilt : new int[]{-40, -24, -15, 15, 24, 40}) {
            check(HiveOpeningAim.estimateOpening(
                    HiveOpeningAim.Alliance.BLUE, 38, new HiveOpeningAim.Vector(0, 0, 2),
                    0, 0, Math.toRadians(90 - tilt), mount, 0, pollen, nectar) == null,
                    "non-endpoint tilt " + tilt + " does not produce an aim");
        }
        check(HiveOpeningAim.estimateOpening(
                HiveOpeningAim.Alliance.BLUE, 37, new HiveOpeningAim.Vector(0, 0, 2), 0, 0, Math.PI / 3,
                mount, 0, pollen, nectar) == null, "red tag ignored");
        HiveOpeningAim.OpeningAim redAudience = HiveOpeningAim.estimateOpening(
                HiveOpeningAim.Alliance.RED, 34, new HiveOpeningAim.Vector(0.2539, 0, 2),
                0, 0, Math.PI / 3, mount, 0, pollen, nectar);
        HiveOpeningAim.OpeningAim redOpposite = HiveOpeningAim.estimateOpening(
                HiveOpeningAim.Alliance.RED, 30, new HiveOpeningAim.Vector(0.2539, 0, 2),
                0, 0, Math.PI / 3, mount, 0, pollen, nectar);
        check(redAudience != null && !redAudience.audienceCellUp, "red audience tag is 34");
        check(redOpposite != null && redOpposite.audienceCellUp, "red opposite tag is 30");
        close(redAudience.opening.y, low.opening.y, "red and blue share tag offsets");
        close(redOpposite.opening.y, low.opening.y, "both red clusters share tag offsets");
        for (int id = 29; id <= 46; id++) {
            check(HiveOpeningAim.Alliance.RED.accepts(id) == (id >= 30 && id <= 37),
                    "red acceptance for ID " + id);
            check(HiveOpeningAim.Alliance.BLUE.accepts(id) == (id >= 38 && id <= 45),
                    "blue acceptance for ID " + id);
        }
    }

    private static void cellSwitchConfirmation() {
        PollenTurret.CellConfirmation confirmation = new PollenTurret.CellConfirmation();
        long ms = 1_000_000;
        check(confirmation.accept(false, 0, true), "first endpoint observation acquires");
        check(!confirmation.accept(true, 40 * ms, true), "single opposite frame holds current CELL");
        check(!confirmation.accept(true, 40 * ms, true), "duplicate exposure does not count");
        check(!confirmation.accept(true, 160 * ms, true), "two exposures cannot confirm");
        check(confirmation.accept(false, 200 * ms, true), "old side discards contrary evidence");
        check(!confirmation.accept(true, 240 * ms, true), "new confirmation starts over");
        confirmation.discardPending();
        check(!confirmation.accept(true, 280 * ms, true), "missing or invalid frame clears pending evidence");
        check(!confirmation.accept(true, 680 * ms, true), "a long gap starts a new confirmation");
        check(!confirmation.accept(true, 720 * ms, true), "second endpoint exposure waits");
        check(!confirmation.accept(true, 760 * ms, true), "short burst waits for the full dwell");
        check(confirmation.accept(true, 800 * ms, true), "consistent endpoint exposures confirm the change");
        check(confirmation.accept(true, 840 * ms, true), "same side tracks normally after confirmation");
        check(!confirmation.accept(false, 880 * ms, true), "a change back also requires confirmation");
        confirmation.clear();
        check(confirmation.accept(false, 920 * ms, true), "explicit reset allows a new initial acquisition");
        check(!confirmation.accept(true, 960 * ms, false), "moving camera supplies first side evidence");
        check(!confirmation.accept(true, 1160 * ms, false), "moving camera supplies second side evidence");
        check(!confirmation.accept(true, 1360 * ms, false), "agreement cannot commit an unsettled field pose");
        check(confirmation.waitingForSettledPose(1360 * ms), "agreed change pauses the servo for a new exposure");
        check(!confirmation.waitingForSettledPose(1911 * ms), "a missing camera result releases the pause");
        check(confirmation.accept(false, 1920 * ms, true), "the original side remains committed until a safe pose");
    }

    private static void check(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }

    private static void close(double actual, double expected, String message) {
        check(Double.isFinite(actual) && Math.abs(actual - expected) < 1e-7,
                message + ": expected " + expected + ", got " + actual);
    }
}
