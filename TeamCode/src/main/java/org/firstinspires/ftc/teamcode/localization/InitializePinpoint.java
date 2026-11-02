package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.LatestPose;

public class InitializePinpoint {

    private final GoBildaPinpointDriver pinpoint;
    private final boolean autonomousPoseRestored;
    private static final String PINPOINT_NAME = "pinpoint";
    private static final DistanceUnit DISTANCE_UNIT = DistanceUnit.MM;
    private static final GoBildaPinpointDriver.GoBildaOdometryPods ENCODER_RESOLUTION = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;
    private static final GoBildaPinpointDriver.EncoderDirection X_POD_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection Y_POD_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final double X_POD_OFFSET_MM = 0;
    private static final double Y_POD_OFFSET_MM = 0;

    public InitializePinpoint(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);

        pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DISTANCE_UNIT);
        pinpoint.setEncoderResolution(ENCODER_RESOLUTION);
        pinpoint.setEncoderDirections(X_POD_DIRECTION, Y_POD_DIRECTION);

        autonomousPoseRestored = LatestPose.getHasPose();
        if (autonomousPoseRestored) {
            pinpoint.setPosX(LatestPose.getX(), DistanceUnit.INCH);
            pinpoint.setPosY(LatestPose.getY(), DistanceUnit.INCH);
            pinpoint.setHeading(LatestPose.getHeading(), AngleUnit.RADIANS);
        }

        pinpoint.update();
    }

    public void update() {
        pinpoint.update();
    }

    public double getXInches() {
        return pinpoint.getPosX(DistanceUnit.INCH);
    }

    public double getYInches() {
        return pinpoint.getPosY(DistanceUnit.INCH);
    }

    public double getHeadingRadians() {
        return pinpoint.getHeading(AngleUnit.RADIANS);
    }

    public double getXPodOffset() {
        return X_POD_OFFSET_MM;
    }

    public double getYPodOffset() {
        return Y_POD_OFFSET_MM;
    }

    public void resetPositionAndIMU() {
        pinpoint.resetPosAndIMU();
    }

    public void resetYaw() {

    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
        telemetry.addData("Autonomous pose restored", autonomousPoseRestored ? "YES" : "NO; using current Pinpoint pose");
        telemetry.addData("Pinpoint X (in)", "%.2f", getXInches());
        telemetry.addData("Pinpoint Y (in)", "%.2f", getYInches());
        telemetry.addData("Pinpoint field heading (deg)", "%.1f", Math.toDegrees(getHeadingRadians()));
    }
}
