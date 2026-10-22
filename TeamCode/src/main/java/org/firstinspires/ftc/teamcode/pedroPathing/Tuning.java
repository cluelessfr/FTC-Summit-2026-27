package org.firstinspires.ftc.teamcode.pedroPathing;

import android.content.Context;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TunerRegistrar;
import org.firstinspires.ftc.ftccommon.external.OnCreate;

import org.firstinspires.ftc.teamcode.pedroPathing.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.Tests;

public final class Tuning {
    private static boolean registered = false;

    private Tuning() {
    }

    // Tuning 1.0.2's @Tuner scanner dereferences putIfAbsent's null return value.
    // Register through the public API at app startup until that dependency is fixed.
    @OnCreate
    public static synchronized void registerTuners(Context context) {
        if (registered) {
            return;
        }
        TunerRegistrar.register("", foresightTuner());
        TunerRegistrar.register("", tests());
        registered = true;
    }

    public static Procedure foresightTuner() {
        return new ForesightTuner(
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig)
        );
    }

    public static Procedure tests() {
        return new Tests(
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig), (hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig)), () -> new Foresight(Constants.foresightConfig));
    }
}
