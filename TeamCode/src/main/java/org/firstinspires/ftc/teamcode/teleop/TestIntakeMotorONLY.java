package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.intakeTest.IntakeMotor;

/** Standalone TeleOp for the shared intake test controls. */
@TeleOp(name = "Intake Motor ONLY")
public class TestIntakeMotorONLY extends LinearOpMode {
    @Override
    public void runOpMode() {
        IntakeMotor intake = new IntakeMotor(hardwareMap);
        try {
            telemetry.addLine("Gamepad 1: A on/off; B reverse; D-pad speed; X 2796 ticks/sec.");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {
                intake.update(gamepad1);
                intake.addTelemetry(telemetry);
                telemetry.update();
            }
        } finally {
            intake.stop();
        }
    }
}
