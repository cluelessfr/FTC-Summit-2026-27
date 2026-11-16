package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.driveTrain.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.flywheelTest.FlywheelMotor;
import org.firstinspires.ftc.teamcode.intakeTest.IntakeMotor;
import org.firstinspires.ftc.teamcode.localization.InitializePinpoint;

/** Runs the shared drivetrain and intake controllers in one TeleOp loop. */
@Disabled
@TeleOp(name = "Intake Motor + Flywheel Motor ONLY", group = "Test")
public class IntakeMotorANDFlywheelMotor extends LinearOpMode {
    @Override
    public void runOpMode() {
        try {
            IntakeMotor intakeMotor = new IntakeMotor(hardwareMap);
            FlywheelMotor flywheelMotor = new FlywheelMotor(hardwareMap);
            try {
                telemetry.addData("Status", "Initialized");
                telemetry.update();
                waitForStart();

                while (opModeIsActive()) {
                    intakeMotor.update(gamepad1);
                    flywheelMotor.update(gamepad1);
                    intakeMotor.addTelemetry(telemetry);
                    flywheelMotor.addTelemetry(telemetry);
                    telemetry.update();
                }
            } finally {
                intakeMotor.stop();
                flywheelMotor.stop();
            }
        } finally {

        }
    }
}
