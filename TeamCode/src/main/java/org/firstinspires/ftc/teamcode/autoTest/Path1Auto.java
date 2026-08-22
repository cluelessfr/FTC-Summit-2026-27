package org.firstinspires.ftc.teamcode.autoTest;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Straight Path Auto", group = "AutoTest")
public class Path1Auto extends LinearOpMode {

    private final Pose startPose = new Pose(72, 72, Math.toRadians(90));
    private final Pose endPose = new Pose(72, 120, Math.toRadians(180));

    private Follower follower;
    private Paths paths;

    public static class Paths {
        public final PathChain path1;

        public Paths(Follower follower, Pose startPose, Pose endPose) {
            path1 = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, endPose))
                    .setLinearHeadingInterpolation(
                            startPose.getHeading(),
                            endPose.getHeading()
                    )
                    .build();
        }
    }

    private Command autoRoutine() {
        return follow(follower, paths.path1, true);
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        paths = new Paths(follower, startPose, endPose);

        telemetry.addLine("Path 1 ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData(
                    "Heading",
                    Math.toDegrees(follower.getPose().getHeading())
            );
            telemetry.update();
        }
    }
}
