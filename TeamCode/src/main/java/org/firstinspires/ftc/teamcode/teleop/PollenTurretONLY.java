package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.turret.HiveOpeningAim.Alliance;
import org.firstinspires.ftc.teamcode.turret.PollenTurret;
import org.firstinspires.ftc.teamcode.localization.InitializePinpoint;

/** Standalone AprilTag aiming test for the POLLEN turret. */
@TeleOp(name = "Pollen Turret ONLY")
public class PollenTurretONLY extends LinearOpMode {
    // Enable after measuring the axis relative to Pinpoint's robot reference.

    @Override
    public void runOpMode() {
        InitializePinpoint pinpoint = new InitializePinpoint(hardwareMap);
        PollenTurret pollenTurret = new PollenTurret(hardwareMap, pinpoint, Alliance.BLUE);

        if (!PollenTurret.hasDefaultCameraMeasurements()) {
            telemetry.addLine("Enter the camera mount measurements in PollenTurret.java.");
            telemetry.update();
            waitForStart();
            return;
        }

        telemetry.addLine("GP1 D-pad right switches the target HIVE between blue and red.");
        pollenTurret.addInitializationTelemetry(telemetry);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        try {
            pollenTurret.start();
            while (opModeIsActive()) {
                pinpoint.update();
                if (gamepad1.dpadRightWasPressed()) pollenTurret.toggleAlliance();
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
