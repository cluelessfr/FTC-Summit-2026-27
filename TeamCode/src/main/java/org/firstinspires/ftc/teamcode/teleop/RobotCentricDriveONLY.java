package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.driveTrain.RobotCentricDrive;

@TeleOp(name = "Robot Centric Drive ONLY", group = "Drivetrain ONLY")
public class RobotCentricDriveONLY extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotCentricDrive drive = new RobotCentricDrive(hardwareMap);
        try {
            telemetry.addData("Status", "Initialized");
            drive.addInitializationTelemetry(telemetry);
            telemetry.update();
            waitForStart();

            if (isStopRequested()) {
                return;
            }

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
