package org.firstinspires.ftc.teamcode.flywheelTest;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FlywheelMotor {
    private static final double POWER_STEP = 0.01;
    private static final double REPEAT_INTERVAL_MS = 100.0;
    private final ElapsedTime adjustmentTimer = new ElapsedTime();
    private int lastAdjustment = 0;
    private double targetPower = 0.0;

    private final DcMotorEx flywheelMotor;
    private int flywheelDirection = 1;
    private boolean flywheelRunning = false;
    private boolean lastA = false;
    private boolean lastB = false;

    public FlywheelMotor(HardwareMap hardwareMap) {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor.setPower(0.0);
    }

    public void update(Gamepad gamepad) {
        if (gamepad.a && !lastA) {
            flywheelRunning = !flywheelRunning;
        }
        lastA = gamepad.a;

        if (gamepad.b && !lastB) {
            flywheelDirection *= -1;
        }
        lastB = gamepad.b;

        int adjustment = (gamepad.dpad_up ? 1 : 0) - (gamepad.dpad_down ? 1 : 0);
        if (adjustment != 0 && (adjustment != lastAdjustment
                || adjustmentTimer.milliseconds() >= REPEAT_INTERVAL_MS)) {
            targetPower = Math.max(0.0, Math.min(1.0, targetPower + adjustment * POWER_STEP));
            adjustmentTimer.reset();
        }
        lastAdjustment = adjustment;

        flywheelMotor.setPower(flywheelRunning ? targetPower * flywheelDirection : 0.0);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("Flywheel: A toggles on/off; B reverses; D-pad up/down changes power.");
        telemetry.addData("Power adjustment", "D-pad up/down: +/- %.0f%%; hold repeats every %.0f ms", POWER_STEP * 100.0, REPEAT_INTERVAL_MS);
        telemetry.addData("Flywheel status", flywheelRunning ? "RUNNING" : "STOPPED");
        telemetry.addData("Flywheel direction", flywheelDirection == 1 ? "FORWARD" : "REVERSE");
        telemetry.addData("Flywheel selected power", "%.0f%%", targetPower * 100.0);
        telemetry.addData("Flywheel commanded power", "%.0f%%", flywheelMotor.getPower() * 100.0);
    }

    public void stop() {
        flywheelRunning = false;
        flywheelMotor.setPower(0.0);
    }
}
