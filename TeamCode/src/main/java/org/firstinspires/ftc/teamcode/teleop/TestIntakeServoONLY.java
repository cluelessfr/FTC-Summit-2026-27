package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.intakeTest.IntakeServos;

/** Standalone TeleOp for the shared intake test controls. */
@TeleOp(name = "Intake Servo ONLY")
public class TestIntakeServoONLY extends LinearOpMode {
    @Override
    public void runOpMode() {
        IntakeServos intakeServos = new IntakeServos(hardwareMap);
        try {
            telemetry.addLine("Gamepad 1 servos: X 0.45 / 0.37; Y 0.52 / 0.00; hold D-pad left/right to adjust.");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {
                intakeServos.update(gamepad1);
                intakeServos.addTelemetry(telemetry);
                telemetry.update();
            }
        } finally {
            intakeServos.stop();
        }
    }
}
