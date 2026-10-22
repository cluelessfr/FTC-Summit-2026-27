package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Shared pose tracking for TeleOps that control their own drivetrain. */
public class GetLatestPoseAndUpdate {

    private final Follower follower;
    private boolean hasPose = false;

    public GetLatestPoseAndUpdate(Follower newFollower) {
        follower = newFollower;

        // Restore once so loop updates can track movement from the saved pose.
        if (LatestPose.getHasPose()) {
            follower.setPose(new Pose(LatestPose.getX(), LatestPose.getY(), LatestPose.getHeading()));
            hasPose = true;
        }
    }

    public void update() {
        // Update position without sending motor power through the follower.
        follower.localizer.update();
    }

    public Pose getPose() {
        return follower.pose();
    }

    public boolean getHasPose() {
        return hasPose;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Autonomous pose restored", hasPose ? "YES" : "NO; using follower pose");
        telemetry.addData("Pose X (in)", getPose().x());
        telemetry.addData("Pose Y (in)", getPose().y());
        telemetry.addData("Pose heading (radians)", getPose().heading());
    }
}
