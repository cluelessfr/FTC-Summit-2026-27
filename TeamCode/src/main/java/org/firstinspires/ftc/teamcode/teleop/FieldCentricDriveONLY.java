package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.driveTrain.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.localization.InitializePinpoint;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@TeleOp(name = "Field Centric Drive ONLY", group = "Drivetrain ONLY")
public class FieldCentricDriveONLY extends LinearOpMode {
    @Override
    public void runOpMode() {
        InitializePinpoint pinpoint = new InitializePinpoint(hardwareMap);
        FieldCentricDrive drive = new FieldCentricDrive(hardwareMap, pinpoint);
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
                pinpoint.update();
                drive.update(gamepad1);
                drive.addTelemetry(telemetry);
                telemetry.update();
            }
        } finally {
            drive.stop();
        }
    }
}
