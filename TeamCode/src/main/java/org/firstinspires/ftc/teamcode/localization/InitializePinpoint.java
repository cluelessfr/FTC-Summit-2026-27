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
    private static final GoBildaPinpointDriver.EncoderDirection X_POD_DIRECTION = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    private static final GoBildaPinpointDriver.EncoderDirection Y_POD_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final double X_POD_OFFSET_MM = -159.5;
    private static final double Y_POD_OFFSET_MM = -132.0;

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
            LatestPose.clearPose();
        } else {
            pinpoint.setPosX(0, DistanceUnit.INCH);
            pinpoint.setPosY(0, DistanceUnit.INCH);
            pinpoint.setHeading(0, AngleUnit.RADIANS);
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

    public double getXVelocityInches() {
        return pinpoint.getVelX(DistanceUnit.INCH);
    }

    public double getYVelocityInches() {
        return pinpoint.getVelY(DistanceUnit.INCH);
    }

    public boolean isReady() {
        return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
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

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Pinpoint", "%s; start: %s", pinpoint.getDeviceStatus(),
                autonomousPoseRestored ? "AUTO" : "ZERO");
        telemetry.addData("Position X / Y (in)", "%.1f / %.1f", getXInches(), getYInches());
        telemetry.addData("Field heading (deg)", "%.1f", Math.toDegrees(getHeadingRadians()));
    }
}
