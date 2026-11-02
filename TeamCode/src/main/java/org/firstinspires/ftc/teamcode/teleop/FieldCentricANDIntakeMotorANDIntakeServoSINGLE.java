package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.driveTrain.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.intakeTest.IntakeMotor;
import org.firstinspires.ftc.teamcode.intakeTest.IntakeServos;
import org.firstinspires.ftc.teamcode.localization.InitializePinpoint;

/** Runs the shared drivetrain and intake controllers in one TeleOp loop. */
@TeleOp(name = "Field Centric Drive + Intake Motor + Intake Servos + Single ONLY", group = "Intake Test")
public class FieldCentricANDIntakeMotorANDIntakeServoSINGLE extends LinearOpMode {
    @Override
    public void runOpMode() {
        InitializePinpoint pinpoint = new InitializePinpoint(hardwareMap);
        FieldCentricDrive drive = new FieldCentricDrive(hardwareMap, pinpoint);
        try {
            IntakeMotor intakeMotor = new IntakeMotor(hardwareMap);
            IntakeServos intakeServos = new IntakeServos(hardwareMap);
            try {
                telemetry.addData("Status", "Initialized");
                drive.addInitializationTelemetry(telemetry);
                telemetry.addLine("Gamepad 1: left stick moves; right stick X turns; A intake on/off; B reverse.");
                telemetry.addLine("Intake runs at 100% power when on.");
                telemetry.addLine("Servos: X 0.45 / 0.37; Y 0.52 / 0.00; hold D-pad left/right to adjust.");
                telemetry.update();
                waitForStart();

                while (opModeIsActive()) {
                    pinpoint.update();
                    drive.update(gamepad1);
                    intakeMotor.update(gamepad1);
                    intakeServos.update(gamepad1);
                    drive.addTelemetry(telemetry);
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
