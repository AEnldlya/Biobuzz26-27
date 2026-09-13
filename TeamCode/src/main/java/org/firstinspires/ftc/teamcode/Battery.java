package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/** Cached battery voltage. A voltage read is its own hub transaction, so refresh every 250 ms. */
public class Battery {
    public static final double NOMINAL_VOLTAGE = 12.5;
    private static final long REFRESH_NS = 250_000_000L;

    private final VoltageSensor sensor;
    private double voltage;
    private long lastReadNs;

    public Battery(HardwareMap hardwareMap) {
        sensor = hardwareMap.voltageSensor.iterator().next();
        voltage = NOMINAL_VOLTAGE;
        read(System.nanoTime());
    }

    public double voltage() {
        long now = System.nanoTime();
        if (now - lastReadNs >= REFRESH_NS) {
            read(now);
        }
        return voltage;
    }

    /** Multiply feedforward power by this so it behaves the same at any battery voltage. */
    public double compensation() {
        return NOMINAL_VOLTAGE / voltage();
    }

    private void read(long now) {
        double reading = sensor.getVoltage();
        // ignore brownout glitches instead of blowing up the compensation
        if (reading > 8.0) {
            voltage = reading;
        }
        lastReadNs = now;
    }
}
