package org.firstinspires.ftc.teamcode.autoTest;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "Pedro Curve Autonomous", group = "Autonomous")
@Configurable
public class PedroCurveAutonomous extends OpMode {
    private static final Pose START_POSE = new Pose(72, 48, Math.toRadians(90));
    private static final Pose CONTROL_POSE = new Pose(120, 72);
    private static final Pose END_POSE = new Pose(72, 96, Math.toRadians(270));

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    @Override
    public void init() {
        Scheduler.reset();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        panelsTelemetry.debug("Follower Busy", follower.isBusy());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public final PathChain path1;

        public Paths(Follower follower) {
            path1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            START_POSE,
                            CONTROL_POSE,
                            END_POSE
                    ))
                    .setLinearHeadingInterpolation(
                            START_POSE.getHeading(),
                            END_POSE.getHeading()
                    )
                    .build();
        }
    }

    private Command autoRoutine() {
        return follow(follower, paths.path1, true);
    }
}
