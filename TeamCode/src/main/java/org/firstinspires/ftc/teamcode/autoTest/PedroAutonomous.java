package org.firstinspires.ftc.teamcode.autoTest;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
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
        public final PathChain path2;

        public Paths(Follower follower) {
            path1 = follower.pathBuilder()
                    .addPath(new BezierLine(START_POSE, PATH_1_END_POSE))
                    .setLinearHeadingInterpolation(
                            START_POSE.getHeading(),
                            PATH_1_END_POSE.getHeading()
                    )
                    .build();

            path2 = follower.pathBuilder()
                    .addPath(new BezierLine(PATH_1_END_POSE, PATH_2_END_POSE))
                    .setLinearHeadingInterpolation(
                            PATH_1_END_POSE.getHeading(),
                            PATH_2_END_POSE.getHeading()
                    )
                    .build();
        }
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, paths.path1, false),
                follow(follower, paths.path2, true)
        );
    }
}
