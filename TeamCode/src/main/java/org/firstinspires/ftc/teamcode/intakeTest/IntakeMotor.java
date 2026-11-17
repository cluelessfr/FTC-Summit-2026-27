package org.firstinspires.ftc.teamcode.intakeTest;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Shared intake test controls; update once per loop without blocking the drivetrain. */
public class IntakeMotor {

    private final DcMotorEx intakeMotor;
    private boolean intakeRunning = false;
    private boolean lastPress = false;

    public IntakeMotor(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0.0);
    }

    public void update(Gamepad gamepad) {
        if (gamepad.cross && !lastPress) {
            intakeRunning = !intakeRunning;
        }
        lastPress = gamepad.cross;

        intakeMotor.setPower(intakeRunning ? 1 : 0.0);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake", "%s", intakeRunning ? "ON" : "OFF");
    }

    public void stop() {
        intakeRunning = false;
        intakeMotor.setPower(0.0);
    }
}
