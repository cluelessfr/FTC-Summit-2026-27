package org.firstinspires.ftc.teamcode.turret;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/** Servo mapping shared by the two independently calibrated turrets. */
public final class TurretServo {
    public static final class Calibration {
        final double home, min, max;
        final double travelDegrees, turretPerServoDegree, rightSign;
        final double minRightDegrees, maxRightDegrees;

        /** Right sign is +1 when increasing servo position turns the turret right. */
        public Calibration(double home, double min, double max, double travelDegrees,
                           double turretPerServoDegree, double rightSign) {
            if (!(0 <= min && min < home && home < max && max <= 1)
                    || !Double.isFinite(travelDegrees) || travelDegrees <= 0
                    || !Double.isFinite(turretPerServoDegree) || turretPerServoDegree <= 0
                    || (rightSign != 1 && rightSign != -1)
                    || !Double.isFinite(travelDegrees * turretPerServoDegree)) {
                throw new IllegalArgumentException("Invalid turret servo calibration.");
            }
            this.home = home;
            this.min = min;
            this.max = max;
            this.travelDegrees = travelDegrees;
            this.turretPerServoDegree = turretPerServoDegree;
            this.rightSign = rightSign;
            double angleAtMinPosition = rightDegrees(min);
            double angleAtMaxPosition = rightDegrees(max);
            minRightDegrees = Math.min(angleAtMinPosition, angleAtMaxPosition);
            maxRightDegrees = Math.max(angleAtMinPosition, angleAtMaxPosition);
        }

        double rightDegrees(double position) {
            return rightSign * (position - home) * travelDegrees * turretPerServoDegree;
        }

        double position(double rightDegrees) {
            return home + rightSign * rightDegrees / (travelDegrees * turretPerServoDegree);
        }

        double nearestReachableRightDegrees(double desiredRightDegrees, double currentRightDegrees) {
            if (!Double.isFinite(desiredRightDegrees)) return Double.NaN;
            double firstReachableTurn = Math.ceil((minRightDegrees - desiredRightDegrees) / 360);
            double lastReachableTurn = Math.floor((maxRightDegrees - desiredRightDegrees) / 360);
            if (firstReachableTurn > lastReachableTurn) return Double.NaN;
            long preferredTurn = Math.round((currentRightDegrees - desiredRightDegrees) / 360);
            double nearestTurn = Math.max(firstReachableTurn, Math.min(lastReachableTurn, preferredTurn));
            return desiredRightDegrees + 360 * nearestTurn;
        }

        double limit(double position) {
            return Math.max(min, Math.min(max, position));
        }
    }

    private final Servo servo;

    TurretServo(HardwareMap hardwareMap, String name) {
        servo = hardwareMap.get(Servo.class, name);
        servo.setDirection(Servo.Direction.FORWARD);
        if (servo instanceof ServoImplEx) {
            ((ServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
    }

    void setPosition(double position) {
        servo.setPosition(position);
    }

    static double moveTowards(double current, double target, double maxChange) {
        double error = target - current;
        if (Math.abs(error) <= maxChange) return target;
        return current + Math.copySign(maxChange, error);
    }
}
