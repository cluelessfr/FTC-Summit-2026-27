package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.flywheelTest.FlywheelMotor;

/** Standalone TeleOp for the shared flywheel test controls. */
@TeleOp(name = "Flywheel Motor ONLY")
public class FlywheelMotorONLY extends LinearOpMode {
    @Override
    public void runOpMode() {
        FlywheelMotor flywheelMotor = new FlywheelMotor(hardwareMap);
        try {
            telemetry.addLine("Gamepad 1: A on/off; B reverse; D-pad up/down adjusts power.");
            flywheelMotor.addTelemetry(telemetry);
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {
                flywheelMotor.update(gamepad1);
                flywheelMotor.addTelemetry(telemetry);
                telemetry.update();
            }
        } finally {
            flywheelMotor.stop();
        }
    }
}
