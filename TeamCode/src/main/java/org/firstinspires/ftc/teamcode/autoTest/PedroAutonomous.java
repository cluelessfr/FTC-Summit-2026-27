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
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.api.Paths.line;

@Autonomous(name = "Pedro Pathing Autonomous")
@Configurable
public class PedroAutonomous extends OpMode {
    private static final Pose START_POSE = new Pose(72, 72, Math.toRadians(90));
    private static final Pose PATH_1_END_POSE = new Pose(72, 96, Math.toRadians(135));
    private static final Pose PATH_2_END_POSE = new Pose(48, 120, Math.toRadians(180));

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
        public final Path path2;

        public Paths() {
            path1 = line(START_POSE, PATH_1_END_POSE).linear(START_POSE, PATH_1_END_POSE);
            path2 = line(PATH_1_END_POSE, PATH_2_END_POSE).linear(PATH_1_END_POSE, PATH_2_END_POSE);
        }
    }

    private static Command follow(Follower follower, Path path, boolean holdEnd) {
        return instant(() -> follower.holdEnd.set(holdEnd))
                .then(com.pedropathing.ivy.pedro.PedroCommands.follow(follower, path));
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, paths.path1, false),
                follow(follower, paths.path2, true)
        );
    }
}
