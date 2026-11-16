package org.firstinspires.ftc.teamcode.flywheelTest;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FlywheelMotor {
    private final DcMotorEx flywheelMotor;
    private boolean flywheelRunning = false;
    private boolean lastPress = false;

    public FlywheelMotor(HardwareMap hardwareMap) {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor.setPower(0.0);
    }

    public void update(Gamepad gamepad) {
        if (gamepad.triangle && !lastPress) {
            flywheelRunning = !flywheelRunning;
        }
        lastPress = gamepad.triangle;


        flywheelMotor.setPower(flywheelRunning ? 1 : 0.0);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("Flywheel: A toggles on/off; B reverses; D-pad up/down changes power.");
        telemetry.addData("Flywheel status", flywheelRunning ? "RUNNING" : "STOPPED");
        telemetry.addData("Flywheel commanded power", "%.0f%%", flywheelMotor.getPower() * 100.0);
    }

    public void stop() {
        flywheelRunning = false;
        flywheelMotor.setPower(0.0);
    }
}
