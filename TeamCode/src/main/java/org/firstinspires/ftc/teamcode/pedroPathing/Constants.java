package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public final class Constants {
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.15689523688729923);
                Controller secondaryTranslationalForward = Controller.proportional(0.05796859771191641);
                Controller primaryTranslationalLateral = Controller.proportional(0.3666694861049963);
                Controller secondaryTranslationalLateral = Controller.proportional(0.13547457752668263);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.015866417516927897));
                c.brake.set(Controller.proportionalFeedforward(0.013486454889388712));

                c.headingFeedback.set(Controller.proportional(1.866700688363891));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.03912404786887095, 0.0065764997283391455));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06454460987914348, 0.05317970009444935));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001180165880939587, 0.001504332085774029));

                c.maxAchievableForwardVelocity.set(67.19513558215264);
                c.maxAchievableStrafeVelocity.set(58.00202414682415);
                c.naturalForwardDeceleration.set(57.515712135493025);
                c.naturalStrafeDeceleration.set(66.52695971599358);
            }
    );

    public static final MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontRightName.set("front_right_motor");
        c.backRightName.set("back_right_motor");
        c.backLeftName.set("back_left_motor");
        c.frontLeftName.set("front_left_motor");

        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static final PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.xPodOffset.set(-2.9852448711245074);
        c.yPodOffset.set(-0.32456141194020915);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    });

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
