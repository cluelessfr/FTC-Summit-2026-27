package org.firstinspires.ftc.teamcode.pedroPathing;

public class LatestPose {
    private static double x = 0;
    private static double y = 0;
    private static double heading = 0;

    private static long latestSavedTime = 0;
    private static boolean hasPose = false;

    public static void setPose(double newX, double newY, double newHeading) {
        x = newX;
        y = newY;
        heading = newHeading;
        latestSavedTime = System.currentTimeMillis();
        hasPose = true;
    }

    public static double getX() {
        return x;
    }

    public static double getY() {
        return y;
    }

    public static double getHeading() {
        return heading;
    }

    public static long getLatestSavedTime() {
        return latestSavedTime;
    }

    public static boolean getHasPose() {
        return hasPose;
    }
}
