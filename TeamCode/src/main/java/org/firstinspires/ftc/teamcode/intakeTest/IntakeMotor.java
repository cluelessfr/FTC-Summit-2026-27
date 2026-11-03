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
    private int intakeDirection = 1;
    private boolean intakeRunning = false;
    private boolean lastA = false;
    private boolean lastB = false;

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

        // Run at full power in the selected direction whenever the intake is on.
        intakeMotor.setPower(intakeRunning ? intakeDirection : 0.0);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake", "%s; %s", intakeRunning ? "ON" : "OFF",
                intakeDirection == 1 ? "FORWARD" : "REVERSE");
    }

    public void stop() {
        intakeRunning = false;
        intakeMotor.setPower(0.0);
    }
}
