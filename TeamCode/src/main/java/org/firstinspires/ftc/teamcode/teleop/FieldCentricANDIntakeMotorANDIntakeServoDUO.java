package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.driveTrain.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.intakeTest.IntakeMotor;
import org.firstinspires.ftc.teamcode.intakeTest.IntakeServos;
import org.firstinspires.ftc.teamcode.localization.InitializePinpoint;

/** Runs the shared drivetrain and intake controllers in one TeleOp loop. */
@TeleOp(name = "Field Centric Drive + Intake Motor + Intake Servos + Duo ONLY", group = "Intake Test")
public class FieldCentricANDIntakeMotorANDIntakeServoDUO extends LinearOpMode {

    @Override
    public void runOpMode() {
        InitializePinpoint pinpoint = new InitializePinpoint(hardwareMap);
        FieldCentricDrive drive = new FieldCentricDrive(hardwareMap, pinpoint);
        try {
            IntakeMotor intakeMotor = new IntakeMotor(hardwareMap);
            IntakeServos intakeServos = new IntakeServos(hardwareMap);
            try {
                telemetry.addData("Status", "Initialized; waiting for Start");
                drive.addInitializationTelemetry(telemetry);
                telemetry.addLine("Gamepad 2 controls intake motor and servos.");
                intakeMotor.addTelemetry(telemetry);
                intakeServos.addTelemetry(telemetry);
                telemetry.update();
                waitForStart();

                while (opModeIsActive()) {
                    pinpoint.update();
                    drive.update(gamepad1);
                    intakeMotor.update(gamepad2);
                    intakeServos.update(gamepad2);
                    telemetry.addData("Status", "Running");
                    drive.addTelemetry(telemetry);
                    telemetry.addLine("Gamepad 2 controls intake motor and servos.");
                    intakeMotor.addTelemetry(telemetry);
                    intakeServos.addTelemetry(telemetry);
                    telemetry.update();
                }
            } finally {
                intakeMotor.stop();
                intakeServos.stop();
            }
        } finally {
            drive.stop();
        }
    }
}
