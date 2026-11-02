package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.driveTrain.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.intakeTest.IntakeMotor;
import org.firstinspires.ftc.teamcode.localization.InitializePinpoint;

/** Runs the shared drivetrain and intake controllers in one TeleOp loop. */
@Disabled
@TeleOp(name = "Field Centric Drive + Intake Motor ONLY", group = "Intake Test")
public class FieldCentricANDIntakeMotor extends LinearOpMode {
    @Override
    public void runOpMode() {
        InitializePinpoint pinpoint = new InitializePinpoint(hardwareMap);
        FieldCentricDrive drive = new FieldCentricDrive(hardwareMap, pinpoint);
        try {
            IntakeMotor intakeMotor = new IntakeMotor(hardwareMap);
            try {
                telemetry.addData("Status", "Initialized");
                drive.addInitializationTelemetry(telemetry);
                telemetry.addLine("Gamepad 1: left stick moves; right stick X turns; A intake on/off; B reverse.");
                telemetry.addLine("Intake runs at 100% power when on.");
                telemetry.update();
                waitForStart();

                while (opModeIsActive()) {
                    drive.update(gamepad1);
                    intakeMotor.update(gamepad1);
                    drive.addTelemetry(telemetry);
                    intakeMotor.addTelemetry(telemetry);
                    telemetry.update();
                }
            } finally {
                intakeMotor.stop();
            }
        } finally {
            drive.stop();
        }
    }
}
