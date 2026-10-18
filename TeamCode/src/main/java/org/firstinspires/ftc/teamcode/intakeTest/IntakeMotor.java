package org.firstinspires.ftc.teamcode.intakeTest;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Shared intake test controls; update once per loop without blocking the drivetrain. */
public class IntakeMotor {

    private final DcMotorEx intakeMotor;
    private int intakePowerPercent = 0;
    private int intakeDirection = 1;
    private boolean intakeRunning = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private int lastSpeedAdjustment = 0;
    private final ElapsedTime intakeAdjustmentTimer = new ElapsedTime();

    public IntakeMotor(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0.0);
    }

    public void update(Gamepad gamepad) {
        if (gamepad.a && !lastA) {
            intakeRunning = !intakeRunning;
        }
        lastA = gamepad.a;

        if (gamepad.b && !lastB) {
            intakeDirection *= -1;
        }
        lastB = gamepad.b;

        // Adjust immediately on press, then every 100 ms while held, without pausing drive.
        int adjustment = (gamepad.dpad_up ? 1 : 0) - (gamepad.dpad_down ? 1 : 0);
        if (adjustment != 0 && (adjustment != lastSpeedAdjustment
                || intakeAdjustmentTimer.milliseconds() >= 100.0)) {
            intakePowerPercent = Math.max(0, Math.min(100, intakePowerPercent + adjustment * 5));
            intakeAdjustmentTimer.reset();
        }
        lastSpeedAdjustment = adjustment;

        // Select full power without changing the run toggle or direction.
        if (gamepad.x) {
            intakePowerPercent = 100;
        }

        intakeMotor.setPower(intakeRunning ? intakePowerPercent / 100.0 * intakeDirection : 0.0);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake status", intakeRunning ? "RUNNING" : "STOPPED");
        telemetry.addData("Intake direction", intakeDirection == 1 ? "FORWARD" : "REVERSE");
        telemetry.addData("Intake selected power", "%d%%", intakePowerPercent);
        telemetry.addData("Intake commanded power", "%.0f%%", intakeMotor.getPower() * 100.0);
    }

    public void stop() {
        intakeRunning = false;
        intakeMotor.setPower(0.0);
    }
}
