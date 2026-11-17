package org.firstinspires.ftc.teamcode.visionTest;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Locale;

/** Reports the 288x288 BACK-bias CPU classifier during INIT and after START. */
@TeleOp(name = "Test: HIVE BACK Bias", group = "Test")
public class HiveBackBiasTest extends LinearOpMode {
    public static final int PIPELINE = 9;
    private static final int MAX_AGE_MS = 1000;

    @Override
    public void runOpMode() throws InterruptedException {
        Limelight3A camera = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(100);
        camera.setPollRateHz(30);
        boolean switched = camera.pipelineSwitch(PIPELINE);
        camera.start();

        double timestamp = Double.NaN;
        long timestampChangedAt = 0;
        try {
            while (!isStopRequested()) {
                LLResult result = camera.getLatestResult();
                long now = System.nanoTime();
                telemetry.addData("Model", "HIVE 288x288 INT8 BACK bias");
                telemetry.addData("Expected slot", PIPELINE);
                if (!switched) telemetry.addLine("Limelight did not acknowledge the pipeline switch.");
                if (result == null || result.getPipelineIndex() != PIPELINE) {
                    telemetry.addData("Orientation", "WAITING FOR MODEL PIPELINE");
                    if (result != null) telemetry.addData("Active slot", result.getPipelineIndex());
                    timestamp = Double.NaN;
                    timestampChangedAt = 0;
                } else {
                    if (Double.isFinite(result.getTimestamp()) && result.getTimestamp() != timestamp) {
                        timestamp = result.getTimestamp();
                        timestampChangedAt = now;
                    }
                    telemetry.addData("Active pipeline type", result.getPipelineType());
                    long age = result.getStaleness();
                    boolean fresh = Double.isFinite(result.getTimestamp()) && age >= 0 && age <= MAX_AGE_MS
                            && timestampChangedAt != 0 && now - timestampChangedAt <= MAX_AGE_MS * 1_000_000L;
                    if (fresh) {
                        showPrediction(result);
                        telemetry.addData("Targeting latency (ms)", "%.1f", result.getTargetingLatency());
                    } else {
                        telemetry.addData("Orientation", "NO FRESH RESULT");
                    }
                }
                telemetry.update();
                sleep(50);
            }
        } finally {
            camera.stop();
        }
    }

    private void showPrediction(LLResult result) {
        String label = null;
        double best = -1;
        for (LLResultTypes.ClassifierResult classification : result.getClassifierResults()) {
            double score = classification.getConfidence();
            if (score >= 0 && score <= 1 && score > best) {
                best = score;
                label = classification.getClassName();
            }
        }

        if (best < 0) {
            telemetry.addData("Orientation", "NO CLASSIFIER RESULT");
            return;
        }
        telemetry.addData("Model score", "%.3f", best);
        String name = label == null ? "" : label.trim().replaceFirst("^\\d+\\s+", "").toLowerCase(Locale.ROOT);
        switch (name) {
            case "left":
            case "right":
            case "front":
            case "back":
                telemetry.addData("Orientation", name.toUpperCase(Locale.ROOT));
                break;
            case "unknown_middle":
                telemetry.addData("Orientation", "UNKNOWN / MIDDLE");
                break;
            default:
                telemetry.addData("Orientation", "UNRECOGNIZED MODEL LABEL");
                telemetry.addData("Raw class", label);
        }
    }
}
