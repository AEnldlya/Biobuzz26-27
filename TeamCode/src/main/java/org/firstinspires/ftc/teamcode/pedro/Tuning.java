package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

/**
 * AutoTune procedures. With the robot on, connect to its Wi-Fi and open
 * http://192.168.43.1:10158 in a browser; run them in order and paste the generated config
 * into Constants.
 */
public class Tuning {
    @Tuner(name = "1. Mecanum Directions")
    public static Procedure mecanum() {
        return new MecanumTuner();
    }

    @Tuner(name = "2. Pinpoint")
    public static Procedure pinpoint() {
        return new PinpointTuner();
    }

    @Tuner(name = "3. Foresight")
    public static Procedure foresight() {
        return new ForesightTuner(Constants::localizer, Constants::drivetrain);
    }

    @Tuner(name = "4. Tests")
    public static Procedure tests() {
        return new Tests(Constants::drivetrain, Constants::localizer, Constants::algorithm);
    }
}
