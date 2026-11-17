package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.flywheelTest.FlywheelMotor;
import org.firstinspires.ftc.teamcode.intakeTest.IntakeMotor;
import org.firstinspires.ftc.teamcode.turret.PollenTurret;

/** Standalone test for the intake, flywheel, and POLLEN turret. */
@TeleOp(name = "Intake + Flywheel + Pollen Turret", group = "Test")
public class IntakeMotorANDFlywheelMotorANDPollenTurret extends LinearOpMode {
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
        PollenTurret pollenTurret = new PollenTurret(hardwareMap);
        telemetry.addLine("Start homes POLLEN and begins AprilTag aiming. Keep the robot stationary.");
        pollenTurret.addInitializationTelemetry(telemetry);
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        try {
            pollenTurret.start();
            while (opModeIsActive()) {
                intakeMotor.update(gamepad1);
                flywheelMotor.update(gamepad1);
                pollenTurret.update();
                intakeMotor.addTelemetry(telemetry);
                flywheelMotor.addTelemetry(telemetry);
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
