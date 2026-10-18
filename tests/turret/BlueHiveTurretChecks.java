package org.firstinspires.ftc.teamcode.turret;

import com.pedropathing.math.Pose;
import java.lang.reflect.Field;
import java.lang.reflect.Method;

/** Run with scripts/check-blue-hive-turret.ps1. No hardware, Android app, or JUnit required. */
public final class BlueHiveTurretChecks {
    private static int checks;

    public static void main(String[] args) throws Exception {
        geometry();
        angleRoutes();
        positionMapping();
        handoff();
        System.out.println("PASS: " + checks + " turret checks (geometry, position mapping, limits, handoff)");
    }

    private static void geometry() {
        BlueHiveTurret.Settings s = calibratedSettings();
        s.hiveX = 10;
        s.hiveY = 10;
        BlueHiveTurret.AimModel model = new BlueHiveTurret.AimModel(s, 0);
        model.plan(new Pose(0, 0, 0));
        close(model.pairX, -5, "rear midpoint X");
        close(model.pairY, 0, "rear midpoint Y");
        close(model.leftAxisX, -5, "left axis X");
        close(model.leftAxisY, 3.5, "left axis Y");
        close(model.rightAxisY, -3.5, "right axis Y");
        close(model.targetDegrees, Math.toDegrees(Math.atan2(10, 15)), "midpoint bearing");
        model.plan(new Pose(0, 0, Math.PI / 2));
        close(model.pairX, 0, "rotated midpoint X");
        close(model.pairY, -5, "rotated midpoint Y");
        close(model.leftAxisX, -3.5, "rotated left axis X");
        close(model.rightAxisX, 3.5, "rotated right axis X");
        close(model.targetDegrees, Math.toDegrees(Math.atan2(15, 10)) - 90, "rotated bearing");
        double target = model.targetDegrees;
        model.plan(new Pose(0, 0, Math.PI / 2 + 12 * Math.PI));
        close(model.targetDegrees, target, "multiple-turn robot heading");
        s.turretAxisSpacingInches = 12;
        model.plan(new Pose(0, 0, Math.PI / 2));
        close(model.targetDegrees, target, "parallel spacing must not change common bearing");
        close(Math.hypot(model.leftAxisX - model.rightAxisX, model.leftAxisY - model.rightAxisY),
                12, "changed axis spacing");

        Pose standalone = BlueHiveTurret.blueWallStartPose(9);
        close(standalone.x(), 135, "standalone wall inset");
        close(standalone.y(), 72, "standalone wall midpoint");
        close(standalone.heading(), Math.PI, "standalone facing hive");
        BlueHiveTurret.AimModel wall = new BlueHiveTurret.AimModel(calibratedSettings(), 0);
        wall.plan(standalone);
        close(wall.targetDegrees, 0, "standalone forward aim");
        for (double inset : new double[] {0, -1, 60, Double.NaN, Double.POSITIVE_INFINITY}) {
            boolean rejected = false;
            try { BlueHiveTurret.blueWallStartPose(inset); }
            catch (IllegalArgumentException expected) { rejected = true; }
            check(rejected, "invalid standalone inset rejected");
        }
        model.plan(null);
        check(!model.targetAvailable, "missing robot pose");
        model.plan(new Pose(Double.NaN, 0, 0));
        check(!model.targetAvailable, "invalid robot pose");
        model.plan(new Pose(15, 10, 0)); // Rear midpoint lies exactly at the hive.
        check(!model.targetAvailable, "bearing undefined at pivot");
    }

    private static BlueHiveTurret.Settings centeredSettings() {
        BlueHiveTurret.Settings s = calibratedSettings();
        s.hiveX = s.hiveY = s.pairCenterForwardInches = s.pairCenterLeftInches = 0;
        return s;
    }

    private static BlueHiveTurret.Settings calibratedSettings() {
        BlueHiveTurret.Settings s = new BlueHiveTurret.Settings();
        s.servoTurnsPerTurretTurn = 1;
        s.servoForwardPosition = 0.5;
        s.increasingPositionCcwSign = 1;
        return s;
    }

    private static Pose poseForBearing(double degrees) {
        double radians = Math.toRadians(degrees);
        return new Pose(-100 * Math.cos(radians), -100 * Math.sin(radians), 0);
    }

    private static void angleRoutes() {
        BlueHiveTurret.Settings s = centeredSettings();
        s.servoTurnsPerTurretTurn = 0.5; // 600-degree turret travel permits equivalent-angle routing.
        BlueHiveTurret.AimModel near = new BlueHiveTurret.AimModel(s, 170);
        near.plan(poseForBearing(-170));
        close(near.targetDegrees, 190, "short 20-degree route across heading wrap");
        s.servoTurnsPerTurretTurn = 0.4; // Servo permits 750 degrees; cable margin restricts it.
        BlueHiveTurret.AimModel positive = new BlueHiveTurret.AimModel(s, 340);
        positive.plan(poseForBearing(350));
        close(positive.targetDegrees, -10, "positive cable boundary chooses untwisted equivalent");
        close(positive.minimumDegrees, -345, "positive range respects cable margin");
        close(positive.maximumDegrees, 345, "negative range respects cable margin");

        BlueHiveTurret.AimModel negative = new BlueHiveTurret.AimModel(s, -340);
        negative.plan(poseForBearing(-350));
        close(negative.targetDegrees, 10, "negative cable boundary equivalent");
        negative.acceptTarget();
        close(negative.commandedDegrees, 10, "accepted position command");

        for (double start : new double[] {-345, -270, -180, -90, 0, 90, 180, 270, 345}) {
            for (int bearing = -720; bearing <= 720; bearing += 7) {
                BlueHiveTurret.AimModel model = new BlueHiveTurret.AimModel(s, start);
                model.plan(poseForBearing(bearing));
                check(model.targetAvailable, "sweep target available");
                check(model.targetDegrees >= -345 && model.targetDegrees <= 345, "sweep cable margin");
                close(Math.sin(Math.toRadians(model.targetDegrees)), Math.sin(Math.toRadians(bearing)),
                        "sweep equivalent sine");
                close(Math.cos(Math.toRadians(model.targetDegrees)), Math.cos(Math.toRadians(bearing)),
                        "sweep equivalent cosine");
                check(model.targetPosition >= 0 && model.targetPosition <= 1, "sweep valid servo position");
            }
        }
    }

    private static void positionMapping() {
        BlueHiveTurret.Settings s = centeredSettings();
        BlueHiveTurret.AimModel model = new BlueHiveTurret.AimModel(s, 0);
        model.plan(poseForBearing(90));
        close(model.targetPosition, 0.8, "1:1 positive position mapping");
        close(model.minimumDegrees, -150, "1:1 minimum travel");
        close(model.maximumDegrees, 150, "1:1 maximum travel");
        model.acceptTarget();
        model.plan(poseForBearing(180));
        check(!model.targetAvailable && model.bearingAvailable, "rear bearing unreachable with 300-degree servo");
        check(Double.isNaN(model.targetPosition), "unreachable target does not emit endpoint command");
        model.acceptTarget();
        close(model.commandedDegrees, 90, "unreachable target retains previous command");
        for (double limit : new double[] {-150, 150}) {
            model.plan(poseForBearing(limit));
            check(model.targetAvailable, "servo endpoint is reachable");
        }
        model.plan(poseForBearing(150.01));
        check(!model.targetAvailable, "beyond servo endpoint is rejected");

        s.servoTurnsPerTurretTurn = 3;
        model.plan(poseForBearing(30));
        close(model.targetPosition, 0.8, "3:1 position mapping");
        close(model.minimumDegrees, -50, "3:1 minimum travel");
        close(model.maximumDegrees, 50, "3:1 maximum travel");
        model.plan(poseForBearing(60));
        check(!model.targetAvailable, "reduction gearing narrows range");
        s.increasingPositionCcwSign = -1;
        model.plan(poseForBearing(30));
        close(model.targetPosition, 0.2, "reversed gear direction mapping");
        close(model.minimumDegrees, -50, "reversed direction lower bound");
        close(model.maximumDegrees, 50, "reversed direction upper bound");

        s.servoTurnsPerTurretTurn = 1;
        s.servoMinimumPosition = 0.1;
        s.servoMaximumPosition = 0.9;
        s.servoForwardPosition = 0.4;
        s.increasingPositionCcwSign = 1;
        model.plan(poseForBearing(30));
        close(model.minimumDegrees, -90, "asymmetric safe minimum");
        close(model.maximumDegrees, 150, "asymmetric safe maximum");
        close(model.targetPosition, 0.5, "offset forward calibration");
        s.increasingPositionCcwSign = -1;
        model.plan(poseForBearing(30));
        close(model.minimumDegrees, -150, "reversed asymmetric minimum");
        close(model.maximumDegrees, 90, "reversed asymmetric maximum");
        close(model.targetPosition, 0.3, "reversed offset forward calibration");

        for (double sign : new double[] {-1, 1}) {
            s.increasingPositionCcwSign = sign;
            for (double ratio : new double[] {0.4, 0.5, 1, 3, 5}) {
                s.servoTurnsPerTurretTurn = ratio;
                for (int angle = -340; angle <= 340; angle += 5) {
                    model.plan(poseForBearing(angle));
                    if (model.targetAvailable) {
                        check(model.targetPosition >= 0.1 && model.targetPosition <= 0.9, "safe position sweep");
                        close(model.angleForPosition(model.targetPosition), model.targetDegrees, "mapping inverse sweep");
                    }
                }
            }
        }
        s.servoTurnsPerTurretTurn = Double.NaN;
        model.plan(poseForBearing(0));
        check(model.bearingAvailable && !model.targetAvailable, "missing calibration computes bearing only");
        s.servoTurnsPerTurretTurn = 1;
        s.servoForwardPosition = 1;
        model.plan(poseForBearing(0));
        check(!model.targetAvailable, "forward outside safe range rejected");
    }

    private static void handoff() throws Exception {
        // Defaults must not access a hardware map or command an unprogrammed servo.
        BlueHiveTurret auto = BlueHiveTurret.forAutonomous(null);
        Pose pose = new Pose(123, 45, 1.2);
        auto.start();
        auto.update(pose);
        Field modelField = BlueHiveTurret.class.getDeclaredField("model");
        modelField.setAccessible(true);
        BlueHiveTurret.AimModel model = (BlueHiveTurret.AimModel) modelField.get(auto);
        check(Double.isNaN(model.commandedDegrees), "uncalibrated default sends no position command");
        model.commandedDegrees = 75;
        auto.stop();
        Method take = BlueHiveTurret.class.getDeclaredMethod("takeHandoff");
        take.setAccessible(true);
        Object saved = take.invoke(null);
        check(saved != null, "stopped auto saves handoff");
        Pose savedPose = (Pose) field(saved, "pose");
        close(savedPose.x(), 123, "handoff actual X");
        close(savedPose.y(), 45, "handoff actual Y");
        close(savedPose.heading(), 1.2, "handoff actual heading");
        close((double) field(saved, "commandedDegrees"), 75, "handoff last commanded angle");
        check(take.invoke(null) == null, "handoff consumed only once");

        auto = BlueHiveTurret.forAutonomous(null);
        auto.start();
        auto.update(pose);
        auto.stop();
        BlueHiveTurret.forAutonomous(null);
        check(take.invoke(null) == null, "new auto clears stale handoff");
        auto = BlueHiveTurret.forAutonomous(null);
        auto.start();
        auto.update(pose);
        auto.update(null);
        auto.stop();
        check(take.invoke(null) == null, "invalid localization clears handoff");
    }

    private static Object field(Object instance, String name) throws Exception {
        Field field = instance.getClass().getDeclaredField(name);
        field.setAccessible(true);
        return field.get(instance);
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
