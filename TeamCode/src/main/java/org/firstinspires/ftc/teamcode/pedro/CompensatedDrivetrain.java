package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Battery;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Custom Pedro drivetrain: the stock mecanum drivetrain with battery voltage compensation.
 *
 * Foresight's feedforwards (coast, brake, heading) are power-per-velocity numbers measured at
 * one battery voltage. Scaling every drive power by NOMINAL_VOLTAGE / battery keeps the robot
 * following the same as the battery sags over a match, instead of undershooting late in a
 * match and overshooting on a fresh pack. Tune Foresight with this drivetrain (Tuning.java uses
 * it), so the tuned values are "at nominal voltage".
 *
 * The scale is capped so a low pack cannot ask for more than MAX_SCALE x power; the motors clip
 * at 1.0 anyway and the extra would only saturate the wheels unevenly.
 */
public class CompensatedDrivetrain implements Drivetrain {
    public static boolean ENABLED = true;
    public static double MAX_SCALE = 1.2;
    public static double MIN_SCALE = 0.85;

    public final Mecanum mecanum;
    private final Battery battery;

    public CompensatedDrivetrain(HardwareMap hardwareMap, MecanumConfig config) {
        mecanum = new Mecanum(hardwareMap, config);
        battery = new Battery(hardwareMap);
    }

    private double scale() {
        if (!ENABLED) {
            return 1.0;
        }
        return Math.max(MIN_SCALE, Math.min(MAX_SCALE, battery.compensation()));
    }

    @Override
    public void drive(DrivePowers powers, boolean normalize) {
        double s = scale();
        if (s == 1.0) {
            mecanum.drive(powers, normalize);
            return;
        }
        mecanum.drive(new DrivePowers(powers.forward() * s, powers.strafe() * s, powers.turn() * s), normalize);
    }

    @Override
    public double maxScaling(DrivePowers powers, DrivePowers direction) {
        return mecanum.maxScaling(powers, direction);
    }

    @Override
    public void stop() {
        mecanum.stop();
    }

    @Override
    public void stop(boolean brake) {
        mecanum.stop(brake);
    }

    @Override
    public double interpolateVelocity(double forward, double strafe, double turn) {
        return mecanum.interpolateVelocity(forward, strafe, turn);
    }

    @Override
    public Map<String, Object> debug() {
        // copy: the stock map may be unmodifiable
        Map<String, Object> map = new LinkedHashMap<>(mecanum.debug());
        map.put("voltageScale", scale());
        return map;
    }

    public double voltage() {
        return battery.voltage();
    }
}
