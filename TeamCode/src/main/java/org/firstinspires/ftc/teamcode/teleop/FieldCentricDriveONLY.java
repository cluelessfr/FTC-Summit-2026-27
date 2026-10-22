package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.driveTrain.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.GetLatestPoseAndUpdate;

@TeleOp(name = "Field Centric Drive ONLY", group = "Drivetrain ONLY")
public class FieldCentricDriveONLY extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);
        FieldCentricDrive drive = new FieldCentricDrive(hardwareMap);
        GetLatestPoseAndUpdate pose = new GetLatestPoseAndUpdate(follower);
        try {
            telemetry.addData("Status", "Initialized");
            drive.addInitializationTelemetry(telemetry);
            telemetry.update();
            waitForStart();

            if (isStopRequested()) {
                return;
            }

            drive.resetHeading();

            while (opModeIsActive()) {
                drive.update(gamepad1);
                drive.addTelemetry(telemetry);
                pose.update();
                pose.addTelemetry(telemetry);
                telemetry.update();
            }
        } finally {
            drive.stop();
        }
    }
}
