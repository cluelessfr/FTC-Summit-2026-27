package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.intakeTest.IntakeMotor;
import org.firstinspires.ftc.teamcode.intakeTest.IntakeServos;

/** Standalone TeleOp for the shared intake test controls. */
@Disabled
@TeleOp(name = "Intake Motor AND Intake Servo")
public class IntakeMotorANDIntakeServo extends LinearOpMode {
    @Override
    public void runOpMode() {
        IntakeServos intakeServos = new IntakeServos(hardwareMap);
        IntakeMotor intakeMotor = new IntakeMotor(hardwareMap);
        try {
            telemetry.addLine("Gamepad 1 servos: X 0.45 / 0.37; Y 0.52 / 0.00; hold D-pad left/right to adjust.");
            telemetry.addLine("Gamepad 1 intake: A on/off; B reverse; 100% power when on.");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {
                intakeServos.update(gamepad1);
                intakeMotor.update(gamepad1);
                intakeServos.addTelemetry(telemetry);
                intakeMotor.addTelemetry(telemetry);
                telemetry.update();
            }
        } finally {
            intakeMotor.stop();
            intakeServos.stop();
        }
    }
}
