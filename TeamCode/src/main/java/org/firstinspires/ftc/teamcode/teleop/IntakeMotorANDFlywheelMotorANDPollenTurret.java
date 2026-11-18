package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.flywheelTest.FlywheelMotor;
import org.firstinspires.ftc.teamcode.intakeTest.IntakeMotor;
import org.firstinspires.ftc.teamcode.turret.PollenTurret;
import org.firstinspires.ftc.teamcode.localization.InitializePinpoint;

/** Standalone test for the intake, flywheel, and POLLEN turret. */
@TeleOp(name = "Intake + Flywheel + Pollen Turret", group = "Test")
public class IntakeMotorANDFlywheelMotorANDPollenTurret extends LinearOpMode {
    // Enable after measuring POLLEN_AXIS_ORIGIN in PollenTurret.java.
    private static final boolean USE_PINPOINT = false;

    @Override
    public void runOpMode() {
        if (!PollenTurret.hasDefaultCameraMeasurements()) {
            telemetry.addLine("Enter the camera mount measurements in PollenTurret.java.");
            telemetry.update();
            waitForStart();
            return;
        }

        IntakeMotor intakeMotor = new IntakeMotor(hardwareMap);
        FlywheelMotor flywheelMotor = new FlywheelMotor(hardwareMap);
        InitializePinpoint pinpoint = USE_PINPOINT ? new InitializePinpoint(hardwareMap) : null;
        PollenTurret pollenTurret = new PollenTurret(hardwareMap, pinpoint);
        telemetry.addLine("Start homes POLLEN and begins AprilTag aiming. Keep the robot stationary.");
        if (USE_PINPOINT) telemetry.addLine("Pinpoint enabled. GP2 A clears the remembered opening.");
        pollenTurret.addInitializationTelemetry(telemetry);
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        try {
            pollenTurret.start();
            while (opModeIsActive()) {
                if (pinpoint != null) pinpoint.update();
                if (pinpoint != null && gamepad2.a) pollenTurret.clearOdometryTarget();
                intakeMotor.update(gamepad1);
                flywheelMotor.update(gamepad1);
                pollenTurret.update();
                intakeMotor.addTelemetry(telemetry);
                flywheelMotor.addTelemetry(telemetry);
                if (pinpoint != null) pinpoint.addTelemetry(telemetry);
                pollenTurret.addTelemetry(telemetry);
                telemetry.update();
                idle();
            }
        } finally {
            intakeMotor.stop();
            flywheelMotor.stop();
            pollenTurret.stop();
        }
    }
}
