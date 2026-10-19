package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.driveTrain.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.intakeTest.IntakeMotor;

/** Runs the shared drivetrain and intake controllers in one TeleOp loop. */
@TeleOp(name = "Field Centric Drive + Intake Motor ONLY", group = "Intake Test")
public class FieldCentricANDIntakeMotor extends LinearOpMode {
    @Override
    public void runOpMode() {
        FieldCentricDrive drive = new FieldCentricDrive(hardwareMap);
        try {
            IntakeMotor intake = new IntakeMotor(hardwareMap);
            try {
                telemetry.addData("Status", "Initialized");
                drive.addInitializationTelemetry(telemetry);
                telemetry.addLine("Gamepad 1: sticks drive; A intake on/off; B reverse;");
                telemetry.addLine("D-pad intake speed; X 2796 ticks/sec.");
                telemetry.update();
                waitForStart();

                while (opModeIsActive()) {
                    drive.update(gamepad1);
                    intake.update(gamepad1);
                    drive.addTelemetry(telemetry);
                    intake.addTelemetry(telemetry);
                    telemetry.update();
                }
            } finally {
                intake.stop();
            }
        } finally {
            drive.stop();
        }
    }
}
