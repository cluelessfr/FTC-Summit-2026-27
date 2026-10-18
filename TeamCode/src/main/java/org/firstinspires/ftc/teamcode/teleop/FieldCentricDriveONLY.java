package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.driveTrain.FieldCentricDrive;

@TeleOp(name = "Field Centric Drive ONLY", group = "Drivetrain ONLY")
public class FieldCentricDriveONLY extends LinearOpMode {
    @Override
    public void runOpMode() {
        FieldCentricDrive drive = new FieldCentricDrive(hardwareMap);
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
                telemetry.update();
            }
        } finally {
            drive.stop();
        }
    }
}
