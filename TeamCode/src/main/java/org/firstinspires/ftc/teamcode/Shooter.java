package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Flywheel velocity PIDF plus the hood (aim) servo.
 *
 * power = (kV * targetRPM + kS) * (12.5 V / battery) + kP * error + kI * integral
 *
 * The feedforward does almost all the work and is voltage compensated, so shots stay the same as
 * the battery drains; PI only cleans up the last bit. When the wheel drops far below target
 * (right after a ball goes through) it runs full power to recover.
 */
public class Shooter {
    // goBILDA 5203 1:1 encoder counts per motor rev. RPM comes from raw ticks/s, so it does not
    // depend on the motor type picked in the robot configuration.
    public static double TICKS_PER_REV = 28.0;

    public static double kP = 0.0005;
    public static double kI = 0.0002;
    public static double kD = 0.0;
    // power per RPM at 12.5 V; measure it with the Shot Tuner (hold BACK)
    public static double kV = 1.0 / 6000.0;
    public static double kS = 0.02;
    public static double I_ZONE_RPM = 250.0;
    public static double MAX_I_POWER = 0.15;
    public static double FULL_POWER_BELOW_RPM = 350.0;
    public static double READY_TOLERANCE_RPM = 100.0;
    public static double RPM_FILTER = 0.5;      // weight of the newest RPM sample

    private static final double MIN_ENCODER_TICKS_PER_SEC = 5.0;
    private static final double ENCODER_CHECK_POWER = 0.35;
    private static final long ENCODER_FAULT_NS = 600_000_000L;

    private final DcMotorEx motor1;
    private final DcMotorEx motor2;
    private final Servo hoodServo;
    private final Battery battery;
    private final PIDFController pidf = new PIDFController(kP, kI, kD, 0.0, 0.0);

    private boolean enabled = false;
    private double targetRPM = 0.0;
    private double rpm = 0.0;
    private double ticksPerSecond = 0.0;
    private double power = 0.0;
    private double hood = Double.NaN;
    private double overridePower = Double.NaN;
    private double lastPowerWritten = Double.NaN;
    private boolean encoderOk = true;
    private long noFeedbackSinceNs = 0;
    private long lastNs = 0;

    public Shooter(HardwareMap hardwareMap, Battery battery) {
        motor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        motor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        hoodServo = hardwareMap.get(Servo.class, "aimServo");
        this.battery = battery;

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // let the flywheel coast down instead of braking it
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        writePower(0.0);
    }

    /** Sets flywheel speed and hood from the shot table. */
    public void setShotDistance(double distanceIn, double rpmTrim) {
        setTargetRPM(ShotTable.rpm(distanceIn) + rpmTrim);
        setHood(ShotTable.hood(distanceIn));
    }

    public void setTargetRPM(double rpm) {
        targetRPM = Math.max(0.0, rpm);
    }

    public void setHood(double position) {
        double clipped = Math.max(0.0, Math.min(1.0, position));
        if (Double.isNaN(hood) || Math.abs(clipped - hood) > 0.002) {
            hoodServo.setPosition(clipped);
            hood = clipped;
        }
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    /** Raw open-loop power for testing (e.g. measuring kV); NaN returns to closed loop. */
    public void setOverridePower(double power) {
        overridePower = power;
    }

    public void update() {
        long now = System.nanoTime();
        double dt = lastNs == 0 ? 0.02 : Math.min(Math.max((now - lastNs) / 1e9, 0.001), 0.1);
        lastNs = now;

        readVelocity();

        if (!Double.isNaN(overridePower)) {
            pidf.reset();
            power = overridePower;
            writePower(power);
            return;
        }

        if (!enabled || targetRPM <= 0.0) {
            pidf.reset();
            power = 0.0;
            writePower(0.0);
            return;
        }

        double feedforward = (kV * targetRPM + kS) * battery.compensation();
        double error = targetRPM - rpm;
        double output;
        if (!encoderOk) {
            output = feedforward;
        } else if (error > FULL_POWER_BELOW_RPM) {
            pidf.reset();
            output = 1.0;
        } else {
            pidf.setGains(kP, kI, kD, 0.0, 0.0);
            pidf.integralZone = I_ZONE_RPM;
            pidf.maxIntegralOutput = MAX_I_POWER;
            output = feedforward + pidf.calculate(error, 0.0, 0.0, dt, false);
        }
        power = Math.max(0.0, Math.min(1.0, output));

        checkEncoder(now);
        writePower(power);
    }

    private void readVelocity() {
        double velocity1 = Math.abs(motor1.getVelocity());
        double velocity2 = Math.abs(motor2.getVelocity());
        // an unplugged encoder reads 0, so average only when both are alive
        if (velocity1 > MIN_ENCODER_TICKS_PER_SEC && velocity2 > MIN_ENCODER_TICKS_PER_SEC) {
            ticksPerSecond = (velocity1 + velocity2) / 2.0;
        } else {
            ticksPerSecond = Math.max(velocity1, velocity2);
        }
        double sample = ticksPerSecond / TICKS_PER_REV * 60.0;
        rpm += RPM_FILTER * (sample - rpm);
    }

    private void checkEncoder(long now) {
        if (ticksPerSecond > MIN_ENCODER_TICKS_PER_SEC) {
            encoderOk = true;
            noFeedbackSinceNs = 0;
        } else if (power >= ENCODER_CHECK_POWER) {
            if (noFeedbackSinceNs == 0) {
                noFeedbackSinceNs = now;
            } else if (now - noFeedbackSinceNs > ENCODER_FAULT_NS) {
                // powered but not turning on the encoder: run on feedforward alone
                encoderOk = false;
            }
        }
    }

    private void writePower(double value) {
        if (Double.isNaN(lastPowerWritten) || Math.abs(value - lastPowerWritten) > 0.005
                || (value == 0.0 && lastPowerWritten != 0.0)) {
            motor1.setPower(value);
            motor2.setPower(value);
            lastPowerWritten = value;
        }
    }

    public void stop() {
        enabled = false;
        targetRPM = 0.0;
        pidf.reset();
        power = 0.0;
        writePower(0.0);
    }

    public boolean atSpeed() {
        if (!enabled || targetRPM <= 0.0) {
            return false;
        }
        return !encoderOk || Math.abs(targetRPM - rpm) <= READY_TOLERANCE_RPM;
    }

    public double getRPM() {
        return rpm;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getTicksPerSecond() {
        return ticksPerSecond;
    }

    public double getPower() {
        return power;
    }

    public double getHood() {
        return hood;
    }

    public boolean hasEncoderFeedback() {
        return encoderOk;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter RPM / target", "%.0f / %.0f %s", rpm, targetRPM, atSpeed() ? "READY" : "");
        telemetry.addData("Shooter power / hood", "%.2f / %.3f", power, hood);
        if (!encoderOk) {
            telemetry.addData("Shooter", "NO ENCODER FEEDBACK - feedforward only");
        }
    }
}
