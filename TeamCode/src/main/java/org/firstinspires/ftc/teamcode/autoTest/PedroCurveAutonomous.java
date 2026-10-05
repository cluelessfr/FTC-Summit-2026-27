package org.firstinspires.ftc.teamcode.autoTest;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.api.Paths.curve;

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
        follower.setPose(START_POSE);
        paths = new Paths();

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
        panelsTelemetry.debug("X", follower.pose().x());
        panelsTelemetry.debug("Y", follower.pose().y());
        panelsTelemetry.debug("Heading", follower.pose().heading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public final Path path1;

        public Paths() {
            path1 = curve(START_POSE, CONTROL_POSE, END_POSE).linear(START_POSE, END_POSE);
        }
    }

    private static Command follow(Follower follower, Path path, boolean holdEnd) {
        return instant(() -> follower.holdEnd.set(holdEnd))
                .then(com.pedropathing.ivy.pedro.PedroCommands.follow(follower, path));
    }

    private Command autoRoutine() {
        return follow(follower, paths.path1, true);
    }
}
