package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.turret.PollenTurret;

/** Standalone AprilTag aiming test for the POLLEN turret. */
@TeleOp(name = "Pollen Turret ONLY")
public class PollenTurretONLY extends LinearOpMode {
    @Override
    public void runOpMode() {
        PollenTurret pollenTurret = new PollenTurret(hardwareMap);

        if (!PollenTurret.hasDefaultCameraMeasurements()) {
            telemetry.addLine("Enter the camera mount measurements in PollenTurret.java.");
            telemetry.update();
            waitForStart();
            return;
        }

        telemetry.addLine("Start homes POLLEN and begins AprilTag aiming. Keep the robot stationary.");
        pollenTurret.addInitializationTelemetry(telemetry);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        try {
            pollenTurret.start();
            while (opModeIsActive()) {
                pollenTurret.update();
                pollenTurret.addTelemetry(telemetry);
                telemetry.update();
                idle();
            }
        } finally {
            pollenTurret.stop();
        }
    }
}
