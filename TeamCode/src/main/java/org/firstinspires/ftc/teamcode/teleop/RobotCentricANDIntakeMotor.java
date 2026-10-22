package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.driveTrain.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.intakeTest.IntakeMotor;

/** Runs the shared drivetrain and intake controllers in one TeleOp loop. */
@TeleOp(name = "Robot Centric Drive + Intake Motor ONLY", group = "Intake Test")
public class RobotCentricANDIntakeMotor extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotCentricDrive drive = new RobotCentricDrive(hardwareMap);
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
