package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.driveTrain.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.flywheelTest.FlywheelMotor;
import org.firstinspires.ftc.teamcode.intakeTest.IntakeMotor;
import org.firstinspires.ftc.teamcode.intakeTest.IntakeServos;
import org.firstinspires.ftc.teamcode.localization.InitializePinpoint;
import org.firstinspires.ftc.teamcode.turret.PollenTurret;

/** Standalone test for the intake, flywheel, and POLLEN turret. */
@TeleOp(name = "Intake + Flywheel + Pollen Turret + Field Centric", group = "Test")
public class IntakeMotorANDFlywheelMotorANDPollenTurretANDFieldCentricANDIntakeServos extends LinearOpMode {
    @Override
    public void runOpMode() {
        if (!PollenTurret.hasDefaultCameraMeasurements()) {
            telemetry.addLine("Enter the camera mount measurements in PollenTurret.java.");
            telemetry.update();
            waitForStart();
            return;
        }

        InitializePinpoint pinpoint = new InitializePinpoint(hardwareMap);
        FieldCentricDrive drive = new FieldCentricDrive(hardwareMap, pinpoint);
        IntakeMotor intakeMotor = new IntakeMotor(hardwareMap);
        IntakeServos intakeServos = new IntakeServos(hardwareMap);
        FlywheelMotor flywheelMotor = new FlywheelMotor(hardwareMap);
        PollenTurret pollenTurret = new PollenTurret(hardwareMap, pinpoint);
        telemetry.addLine("GP1 D-pad right switches the target HIVE between blue and red.");
        pollenTurret.addInitializationTelemetry(telemetry);
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        try {
            pollenTurret.start();
            while (opModeIsActive()) {
                pinpoint.update();
                drive.update(gamepad1);
                intakeMotor.update(gamepad1);
                intakeServos.update(gamepad1);
                flywheelMotor.update(gamepad1);
                if (gamepad1.dpadRightWasPressed()) pollenTurret.toggleAlliance();
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
