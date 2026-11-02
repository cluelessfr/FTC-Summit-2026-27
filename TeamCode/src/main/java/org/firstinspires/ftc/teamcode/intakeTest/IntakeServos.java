package org.firstinspires.ftc.teamcode.intakeTest;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeServos {

    private final Servo SERVO_1;
    private final Servo SERVO_2;
    private static final double POSITION_STEP = 0.01;
    private static final double REPEAT_INTERVAL_MS = 100.0;
    private final ElapsedTime adjustmentTimer = new ElapsedTime();
    private int lastAdjustment = 0;
    private boolean lastX = false;
    private boolean nextXPositionIs045 = true;
    private boolean lastY = false;
    private boolean nextYPositionIs052 = true;
    private final double fullyBack = 0.13;

    public IntakeServos(HardwareMap hardwareMap) {
        String servo_1_name = "servo1";
        SERVO_1 = hardwareMap.get(Servo.class, servo_1_name);
        String servo_2_name = "servo2";
        SERVO_2 = hardwareMap.get(Servo.class, servo_2_name);
        SERVO_2.setDirection(Servo.Direction.REVERSE);
        SERVO_1.setPosition(fullyBack);
        SERVO_2.setPosition(fullyBack);
    }

    public void update(Gamepad gamepad) {
        boolean xPressed = gamepad.x && !lastX;
        boolean yPressed = gamepad.y && !lastY;
        lastX = gamepad.x;
        lastY = gamepad.y;

        if (yPressed) {
            double position = fullyBack;
            SERVO_1.setPosition(position);
            SERVO_2.setPosition(position);
            nextYPositionIs052 = !nextYPositionIs052;
            lastAdjustment = 0;
            return;
        }

        if (xPressed) {
            double nectarHeight = 0.2;
            double pollenHeight = 0.3;
            double position = nextXPositionIs045 ? nectarHeight : pollenHeight;
            SERVO_1.setPosition(position);
            SERVO_2.setPosition(position);
            nextXPositionIs045 = !nextXPositionIs045;
            lastAdjustment = 0;
            return;
        }

        int adjustment = (gamepad.dpad_right ? 1 : 0) - (gamepad.dpad_left ? 1 : 0);
        if (adjustment != 0 && (adjustment != lastAdjustment
                || adjustmentTimer.milliseconds() >= REPEAT_INTERVAL_MS)) {
            double change = adjustment * POSITION_STEP;
            SERVO_1.setPosition(Math.max(0.0, Math.min(1.0, SERVO_1.getPosition() + change)));
            SERVO_2.setPosition(Math.max(0.0, Math.min(1.0, SERVO_2.getPosition() + change)));
            adjustmentTimer.reset();
        }
        lastAdjustment = adjustment;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("Servos: X alternates nectar 0.20 / pollen 0.30; Y returns to 0.13.");
        telemetry.addData("Servo adjustment", "D-pad left/right: -/+ %.2f; hold repeats every %.0f ms", POSITION_STEP, REPEAT_INTERVAL_MS);
        telemetry.addLine("Servo positions are commands (0-1); servo2 direction is reversed.");
        telemetry.addData("Servo 1 commanded position (0-1)", "%.2f", SERVO_1.getPosition());
        telemetry.addData("Servo 2 commanded position (0-1)", "%.2f", SERVO_2.getPosition());
    }

    public void stop() {
        SERVO_1.setPosition(fullyBack);
        SERVO_2.setPosition(fullyBack);
    }
}
