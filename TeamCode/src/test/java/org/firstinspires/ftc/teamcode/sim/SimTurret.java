package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.Turret;

import java.util.Random;

/**
 * Two continuous-rotation servos on one ring, each with an analog position wire.
 *
 * Each servo: power below the dead zone does nothing; above it the shaft speed follows a
 * first-order lag toward power * maxServoDegS. The ring turns at the average of the two servo
 * speeds (they are geared together; if they disagree they fight and the ring gets the mean).
 * Each wire outputs 0..3.3 V over one shaft revolution with a little noise, and can be told to
 * glitch (read garbage) to test the two-wire fusion.
 *
 * Ring angle convention matches Turret: degrees from robot forward, positive = right.
 */
public class SimTurret {
    public double maxServoDegS = 380.0;
    public double deadZone = 0.05;
    public double tau = 0.05;
    public double noiseVolts = 0.004;
    public double gearRatio = Turret.GEAR_RATIO;
    /** raw degrees near the top of the range where a real analog output is undefined */
    public double wireDeadZoneDeg = 3.0;

    private final SimDevices.SimCRServo servo1;
    private final SimDevices.SimCRServo servo2;
    private final Random random = new Random(3);
    private double speed1 = 0, speed2 = 0;   // servo shaft deg/s
    private double ringDeg = 0.0;            // true turret angle
    private double shaft1 = 0.0, shaft2 = 0.0;
    private boolean glitchWire2 = false;

    public SimTurret(SimDevices.SimCRServo servo1, SimDevices.SimCRServo servo2, double startRingDeg) {
        this.servo1 = servo1;
        this.servo2 = servo2;
        ringDeg = startRingDeg;
        shaft1 = ringDeg * gearRatio;
        shaft2 = ringDeg * gearRatio;
    }

    private double freeSpeed(double power) {
        double mag = Math.abs(power);
        if (mag < deadZone) {
            return 0.0;
        }
        return Math.copySign((mag - deadZone) / (1.0 - deadZone) * maxServoDegS, power);
    }

    public void step(double dt) {
        double a = Math.min(1.0, dt / tau);
        // Turret writes servo2 with SERVO2_DIRECTION already applied; both are mounted the same
        // way in the sim, so equal powers turn the ring together
        speed1 += (freeSpeed(servo1.effectivePower() * Turret.POWER_DIRECTION) - speed1) * a;
        speed2 += (freeSpeed(servo2.effectivePower() * Turret.POWER_DIRECTION * Turret.SERVO2_DIRECTION) - speed2) * a;
        double ringSpeed = 0.5 * (speed1 + speed2) / gearRatio;
        ringDeg += ringSpeed * dt;
        shaft1 = ringDeg * gearRatio;
        shaft2 = ringDeg * gearRatio;
    }

    private double wireVolts(double shaftDeg, boolean glitch) {
        double raw = ((shaftDeg % 360.0) + 360.0) % 360.0;
        if (glitch) {
            return random.nextDouble() * 3.3;
        }
        if (raw > 360.0 - wireDeadZoneDeg) {
            // undefined region: the output collapses toward 0 V
            return random.nextDouble() * 0.05;
        }
        return raw / 360.0 * Turret.ANALOG_MAX_VOLTAGE + random.nextGaussian() * noiseVolts;
    }

    public double wire1Volts() {
        return wireVolts(shaft1 * Turret.ENCODER_DIRECTION, false);
    }

    public double wire2Volts() {
        return wireVolts(shaft2 * Turret.ENCODER2_DIRECTION, glitchWire2);
    }

    public void setGlitchWire2(boolean glitch) {
        glitchWire2 = glitch;
    }

    /** true turret angle, deg, positive = right */
    public double ringDeg() {
        return ringDeg;
    }

    public double ringSpeedDegS() {
        return 0.5 * (speed1 + speed2) / gearRatio;
    }
}
