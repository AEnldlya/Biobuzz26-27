package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.ShotTable;

/**
 * Flywheel: two motors on one wheel. Free speed is power * 6000 RPM scaled by battery
 * voltage, the wheel follows it with a first-order lag, and each ball through it costs
 * rpmDropPerBall. The encoders report ticks per second at Shooter.TICKS_PER_REV.
 *
 * Range: the ShotTable is treated as truth (an RPM lands a ball at the distance the table maps
 * to it), so a shot is on range when the flywheel was at the table speed for the real distance.
 */
public class SimShooter {
    public double freeRpmPerPowerAt12_5V = 6000.0;
    public double tau = 0.45;
    public double rpmDropPerBall = 250.0;
    public double frictionRpm = 60.0;

    private final SimDevices.SimMotor motor1;
    private final SimDevices.SimMotor motor2;
    private final SimDevices.SimVoltageSensor battery;
    private double rpm = 0.0;
    private double ticks = 0.0;

    public SimShooter(SimDevices.SimMotor motor1, SimDevices.SimMotor motor2, SimDevices.SimVoltageSensor battery) {
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.battery = battery;
        motor1.ticksPerSecond = this::ticksPerSecond;
        motor2.ticksPerSecond = this::ticksPerSecond;
        motor1.ticks = () -> ticks;
        motor2.ticks = () -> ticks;
    }

    public void step(double dt) {
        // both motors drive the same wheel and are mounted mirrored, which is what the REVERSE
        // direction on one of them is for: the commanded powers are what spins the wheel
        double power = 0.5 * (motor1.getPower() + motor2.getPower());
        double free = Math.max(0.0, power * freeRpmPerPowerAt12_5V * battery.volts / 12.5 - frictionRpm);
        if (power <= 0.0) {
            free = 0.0;
        }
        rpm += (free - rpm) * Math.min(1.0, dt / tau);
        rpm = Math.max(0.0, rpm);
        ticks += rpm / 60.0 * Shooter.TICKS_PER_REV * dt;
    }

    private double ticksPerSecond() {
        return rpm / 60.0 * Shooter.TICKS_PER_REV;
    }

    public double rpm() {
        return rpm;
    }

    /** A ball went through: the wheel slows. */
    public void ballThrough() {
        rpm = Math.max(0.0, rpm - rpmDropPerBall);
    }

    /** Distance the current flywheel speed carries a ball, by inverting ShotTable.RPM. */
    public double rangeForRpm(double rpmNow) {
        double[] d = ShotTable.DISTANCE_IN;
        double[] r = ShotTable.RPM;
        if (rpmNow <= r[0]) {
            return d[0] * rpmNow / r[0];
        }
        for (int i = 1; i < r.length; i++) {
            if (rpmNow <= r[i]) {
                double t = (rpmNow - r[i - 1]) / (r[i] - r[i - 1]);
                return d[i - 1] + t * (d[i] - d[i - 1]);
            }
        }
        int last = r.length - 1;
        return d[last] + (rpmNow - r[last]) / (r[last] - r[last - 1]) * (d[last] - d[last - 1]);
    }
}
