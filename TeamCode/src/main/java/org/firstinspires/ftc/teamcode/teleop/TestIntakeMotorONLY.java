package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.intakeTest.IntakeMotor;

/** Standalone TeleOp for the shared intake test controls. */
@TeleOp(name = "Intake Motor ONLY")
public class TestIntakeMotorONLY extends LinearOpMode {
    @Override
    public void runOpMode() {
        IntakeMotor intakeMotor = new IntakeMotor(hardwareMap);
        try {
            telemetry.addLine("Gamepad 1: A on/off; B reverse; intake runs at 100% power.");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {
                intakeMotor.update(gamepad1);
                intakeMotor.addTelemetry(telemetry);
                telemetry.update();
            }
        } finally {
            intakeMotor.stop();
        }
    }
}
