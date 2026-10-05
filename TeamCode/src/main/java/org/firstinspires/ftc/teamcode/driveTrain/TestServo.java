package org.firstinspires.ftc.teamcode.driveTrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Standard Servo Control", group = "Linear OpMode")
public class TestServo extends LinearOpMode {

    // 1. Declare the servo
    private Servo clawServo;

    @Override
    public void runOpMode() {
        // 2. Map the servo to your Driver Hub configuration name
        clawServo = hardwareMap.get(Servo.class, "servo");

        // 3. Optional: Set a starting position during initialization
        clawServo.setPosition(0.5); // Midpoint

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 4. Control positions using gamepad buttons
            if (gamepad1.a) {
                clawServo.setPosition(0.0); // Move to minimum position
            } else if (gamepad1.b) {
                clawServo.setPosition(1.0); // Move to maximum position
            } else if (gamepad1.x) {
                clawServo.setPosition(0.5); // Return to midpoint
            }

            // Send feedback back to the Driver Hub
            telemetry.addData("Servo Position", clawServo.getPosition());
            telemetry.update();
        }
    }
}