package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Field-relative turret aim from odometry only (Pinpoint dead wheels + its IMU, read through
 * Pedro). No camera.
 *
 * Every loop: take the robot pose and velocity, find the bearing from the turret pivot to the
 * upward CELL of our HIVE, and drive the turret there with PIDF. The feedforward is the turret
 * speed needed to cancel the robot's own spinning and driving, so the turret stays locked on
 * while the robot moves instead of lagging behind it.
 *
 * Turret angles are degrees from robot forward, positive = RIGHT (clockwise), same as the old
 * turret code and the trim buttons.
 *
 * Hardware: a servo in continuous-rotation ("infinite turn") mode, so setPower() sets its
 * SPEED, plus the servo's position wire on an analog input. The analog voltage is the servo
 * output shaft angle, 0 to ANALOG_MAX_VOLTAGE per servo revolution, so it wraps every turn of
 * the servo; the code unwraps it (counts turns) and divides by GEAR_RATIO to get the turret
 * angle. With a 1:1 servo the reading is absolute: set FORWARD_RAW_DEG to the raw angle read
 * when the turret faces forward and it can start anywhere. Otherwise the turret must face
 * forward when the OpMode inits unless an angle is handed over from AUTO with
 * setAngleReference().
 */
public class Turret {
    // ---- hardware ----
    public static String SERVO_NAME = "turretServo";     // CRServo (continuous rotation mode)
    public static String FEEDBACK_NAME = "turretEncoder"; // analog input on the position wire
    public static double ANALOG_MAX_VOLTAGE = 3.3;       // voltage at a full servo revolution
    public static double GEAR_RATIO = 1.0;               // servo revs per turret rev
    /** raw feedback angle (deg, before direction) with the turret facing forward; NaN = the
     *  turret faces forward at init (required when GEAR_RATIO != 1) */
    public static double FORWARD_RAW_DEG = Double.NaN;
    public static double ENCODER_DIRECTION = 1.0;        // makes a right turn read positive
    public static double POWER_DIRECTION = 1.0;          // positive power turns the turret right
    public static double MIN_ANGLE_DEG = -180.0;         // cable limits relative to forward
    public static double MAX_ANGLE_DEG = 180.0;
    /** weight of each new analog sample (1 = no filtering); analog reads carry a little noise */
    public static double ANGLE_FILTER = 0.7;
    // turret pivot relative to the robot's odometry tracking center, inches
    public static double PIVOT_FORWARD_IN = 0.0;
    public static double PIVOT_LEFT_IN = 0.0;

    // ---- PIDF, in power and degrees ----
    public static double kP = 0.007;
    public static double kI = 0.0005;
    public static double kD = 0.0004;
    // power per deg/s at the turret: a CR servo runs ~300-400 deg/s at full power, divided by
    // GEAR_RATIO. Measure it: full power, read "Turret vel" in telemetry, kV = 1 / that.
    public static double kV = 1.0 / 300.0;
    public static double kS = 0.06;             // gets past the servo's dead zone around 0 power
    public static double I_ZONE_DEG = 5.0;
    public static double MAX_I_POWER = 0.1;
    public static double MAX_POWER = 0.9;
    public static double DEADBAND_DEG = 0.4;    // settled inside this: stop commanding, no chatter
    // once settled, stay settled until the error grows past this (hysteresis, no buzzing at
    // the deadband edge where kS would kick the turret back and forth)
    public static double UNSETTLE_DEG = 1.0;
    public static double ON_TARGET_DEG = 1.5;
    public static double VELOCITY_FILTER = 0.5; // weight of the newest turret velocity sample
    // when the target is only this far past a limit, stay pinned instead of swinging 360
    public static double WRAP_HYSTERESIS_DEG = 15.0;
    public static double TRIM_LIMIT_DEG = 45.0;

    // aim where the CELL will be relative to us when the ball arrives (uses
    // ShotTable.TIME_OF_FLIGHT_S); 0 turns shoot-while-moving compensation off
    public static double LEAD_GAIN = 1.0;

    public enum Mode {
        AUTO_AIM,
        HOLD_FORWARD,
        MANUAL,
        OFF
    }

    private final CRServo servo;
    private final AnalogInput feedback;
    private final Battery battery;
    private final PIDFController pidf = new PIDFController(kP, kI, kD, kV, kS);

    private Mode mode = Mode.OFF;
    private Alliance alliance = Alliance.RED;
    private Field.CellSide upCell = Field.startingUpCell(Alliance.RED);
    private double rawDeg = 0.0;          // last analog reading, 0..360 at the servo
    private double unwrappedDeg = 0.0;    // servo angle with turns counted, from init
    private double zeroDeg = 0.0;         // unwrappedDeg value that is turret forward
    private double trimDeg = 0.0;
    private double manualPower = 0.0;

    private double angleDeg = 0.0;
    private double lastAngleDeg = 0.0;
    private double velocityDegPerSec = 0.0;
    private double aimAngleDeg = 0.0;
    private double aimRateDegPerSec = 0.0;
    private double targetDeg = 0.0;
    private double targetRateDegPerSec = 0.0;
    private double errorDeg = 0.0;
    private double power = 0.0;
    private double lastPowerWritten = Double.NaN;
    private double distanceIn = Double.NaN;
    private double trueDistanceIn = Double.NaN;
    private boolean poseValid = false;
    private boolean limited = false;
    private boolean settled = false;
    private long lastNs;

    public Turret(HardwareMap hardwareMap, Battery battery) {
        servo = hardwareMap.get(CRServo.class, SERVO_NAME);
        feedback = hardwareMap.get(AnalogInput.class, FEEDBACK_NAME);
        this.battery = battery;

        writePower(0.0);

        rawDeg = readRawDeg();
        unwrappedDeg = rawDeg;
        if (GEAR_RATIO == 1.0 && !Double.isNaN(FORWARD_RAW_DEG)) {
            // 1:1 servo: the reading is absolute, so the turret can start anywhere
            setAngleReference(wrapDeg(rawDeg - FORWARD_RAW_DEG) * ENCODER_DIRECTION);
        } else {
            setAngleReference(0.0);
        }
        lastNs = System.nanoTime();
    }

    /** Tell the turret its current angle (e.g. the angle AUTO ended at) instead of forward = 0. */
    public void setAngleReference(double currentAngleDeg) {
        zeroDeg = unwrappedDeg - currentAngleDeg * GEAR_RATIO * ENCODER_DIRECTION;
        angleDeg = currentAngleDeg;
        lastAngleDeg = currentAngleDeg;
        velocityDegPerSec = 0.0;
    }

    public void update(Pose pose, Velocity velocity) {
        long now = System.nanoTime();
        double dt = Math.min(Math.max((now - lastNs) / 1e9, 0.001), 0.1);
        lastNs = now;

        double measured = readAngleDeg();
        angleDeg += ANGLE_FILTER * (measured - angleDeg);
        velocityDegPerSec += VELOCITY_FILTER * ((angleDeg - lastAngleDeg) / dt - velocityDegPerSec);
        lastAngleDeg = angleDeg;

        poseValid = pose != null && isFinite(pose.x()) && isFinite(pose.y()) && isFinite(pose.heading());
        if (poseValid) {
            computeAim(pose, velocity);
        }

        double desired;
        double desiredRate = 0.0;
        switch (mode) {
            case AUTO_AIM:
                if (poseValid) {
                    desired = aimAngleDeg + trimDeg;
                    desiredRate = aimRateDegPerSec;
                } else {
                    desired = angleDeg;
                }
                break;
            case HOLD_FORWARD:
                desired = 0.0;
                break;
            case MANUAL:
                pidf.reset();
                settled = false;
                applyPower(manualPower);
                return;
            default:
                pidf.reset();
                settled = false;
                applyPower(0.0);
                return;
        }

        targetDeg = chooseReachable(desired, angleDeg);
        limited = Math.abs(wrapDeg(targetDeg - desired)) > 0.5;
        targetRateDegPerSec = limited ? 0.0 : desiredRate;
        errorDeg = targetDeg - angleDeg;

        boolean targetStill = Math.abs(targetRateDegPerSec) < 2.0;
        if (settled) {
            settled = targetStill && Math.abs(errorDeg) < Math.max(UNSETTLE_DEG, DEADBAND_DEG);
        } else {
            settled = targetStill && Math.abs(errorDeg) < DEADBAND_DEG;
        }
        double output;
        if (settled) {
            pidf.reset();
            output = 0.0;
        } else {
            pidf.setGains(kP, kI, kD, kV, kS);
            pidf.integralZone = I_ZONE_DEG;
            pidf.maxIntegralOutput = MAX_I_POWER;
            output = pidf.calculate(errorDeg, targetRateDegPerSec - velocityDegPerSec,
                    targetRateDegPerSec, dt, true);
            output *= battery.compensation();
        }
        applyPower(Math.max(-MAX_POWER, Math.min(MAX_POWER, output)));
    }

    private void computeAim(Pose pose, Velocity velocity) {
        double heading = pose.heading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double offsetX = PIVOT_FORWARD_IN * cos - PIVOT_LEFT_IN * sin;
        double offsetY = PIVOT_FORWARD_IN * sin + PIVOT_LEFT_IN * cos;
        double pivotX = pose.x() + offsetX;
        double pivotY = pose.y() + offsetY;

        // field-frame velocity of the pivot: robot velocity plus spin carrying the offset around
        double omega = 0.0;
        double vx = 0.0;
        double vy = 0.0;
        if (velocity != null && isFinite(velocity.vx) && isFinite(velocity.vy) && isFinite(velocity.omega)) {
            omega = velocity.omega;
            vx = velocity.vx - omega * offsetY;
            vy = velocity.vy + omega * offsetX;
        }

        Pose cell = Field.cellAimPoint(alliance, upCell);
        trueDistanceIn = Math.hypot(cell.x() - pivotX, cell.y() - pivotY);

        double aimX = cell.x();
        double aimY = cell.y();
        if (LEAD_GAIN != 0.0) {
            // two passes: flight time depends on distance, which depends on the lead
            for (int i = 0; i < 2; i++) {
                double flightTime = ShotTable.timeOfFlight(Math.hypot(aimX - pivotX, aimY - pivotY)) * LEAD_GAIN;
                aimX = cell.x() - vx * flightTime;
                aimY = cell.y() - vy * flightTime;
            }
        }

        double dx = aimX - pivotX;
        double dy = aimY - pivotY;
        double rangeSquared = Math.max(dx * dx + dy * dy, 1.0);
        distanceIn = Math.sqrt(rangeSquared);

        // bearing relative to robot forward is counter-clockwise; turret angles are clockwise
        double relative = Angle.normalizeSigned(Math.atan2(dy, dx) - heading);
        aimAngleDeg = -Math.toDegrees(relative);

        // d(atan2(dy, dx))/dt with the pivot moving at (vx, vy), minus the robot's own spin
        double bearingRate = (dy * vx - dx * vy) / rangeSquared;
        aimRateDegPerSec = -Math.toDegrees(bearingRate - omega);
    }

    private double chooseReachable(double desired, double current) {
        double best = Double.NaN;
        double bestGap = Double.POSITIVE_INFINITY;
        for (int k = -2; k <= 2; k++) {
            double candidate = desired + 360.0 * k;
            if (candidate > MAX_ANGLE_DEG) {
                if (candidate - MAX_ANGLE_DEG <= WRAP_HYSTERESIS_DEG && current > MAX_ANGLE_DEG - WRAP_HYSTERESIS_DEG) {
                    return MAX_ANGLE_DEG;
                }
                continue;
            }
            if (candidate < MIN_ANGLE_DEG) {
                if (MIN_ANGLE_DEG - candidate <= WRAP_HYSTERESIS_DEG && current < MIN_ANGLE_DEG + WRAP_HYSTERESIS_DEG) {
                    return MIN_ANGLE_DEG;
                }
                continue;
            }
            double gap = Math.abs(candidate - current);
            if (gap < bestGap) {
                bestGap = gap;
                best = candidate;
            }
        }
        return Double.isNaN(best) ? Math.max(MIN_ANGLE_DEG, Math.min(MAX_ANGLE_DEG, desired)) : best;
    }

    private void applyPower(double requested) {
        if ((angleDeg >= MAX_ANGLE_DEG && requested > 0.0) || (angleDeg <= MIN_ANGLE_DEG && requested < 0.0)) {
            requested = 0.0;
        }
        power = requested;
        writePower(POWER_DIRECTION * requested);
    }

    private void writePower(double value) {
        // skip redundant writes; each one is a hub transaction
        if (Double.isNaN(lastPowerWritten) || Math.abs(value - lastPowerWritten) > 0.005
                || (value == 0.0 && lastPowerWritten != 0.0)) {
            servo.setPower(value);
            lastPowerWritten = value;
        }
    }

    /** servo shaft angle 0..360 from the position wire */
    private double readRawDeg() {
        double volts = feedback.getVoltage();
        double deg = volts / ANALOG_MAX_VOLTAGE * 360.0;
        return Math.max(0.0, Math.min(360.0, deg));
    }

    /**
     * Unwraps the analog angle (it jumps 360 -> 0 once per servo turn; the servo never moves
     * anywhere near 180 deg between two loops, so the shortest step is the real one) and
     * converts to turret degrees from forward.
     */
    private double readAngleDeg() {
        double raw = readRawDeg();
        unwrappedDeg += wrapDeg(raw - rawDeg);
        rawDeg = raw;
        return (unwrappedDeg - zeroDeg) * ENCODER_DIRECTION / GEAR_RATIO;
    }

    /** raw position-wire angle at the servo, for finding FORWARD_RAW_DEG */
    public double getRawDeg() {
        return rawDeg;
    }

    private static double wrapDeg(double degrees) {
        return Math.toDegrees(Angle.normalizeSigned(Math.toRadians(degrees)));
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    public void setMode(Mode mode) {
        if (mode != this.mode) {
            pidf.reset();
        }
        this.mode = mode;
    }

    public Mode getMode() {
        return mode;
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public void setUpCell(Field.CellSide upCell) {
        this.upCell = upCell;
    }

    /** Call when the HIVE tips: the other CELL is now facing up. */
    public void flipUpCell() {
        upCell = upCell.flipped();
    }

    public Field.CellSide getUpCell() {
        return upCell;
    }

    /** Positive = aim further right. */
    public void adjustTrim(double deltaDeg) {
        trimDeg = Math.max(-TRIM_LIMIT_DEG, Math.min(TRIM_LIMIT_DEG, trimDeg + deltaDeg));
    }

    public void resetTrim() {
        trimDeg = 0.0;
    }

    public double getTrimDeg() {
        return trimDeg;
    }

    /** Only used in MANUAL mode; positive = right. */
    public void setManualPower(double power) {
        manualPower = power;
    }

    public void stop() {
        setMode(Mode.OFF);
        applyPower(0.0);
    }

    public boolean isOnTarget() {
        return mode == Mode.AUTO_AIM && poseValid && !limited && Math.abs(errorDeg) <= ON_TARGET_DEG;
    }

    /** Distance to the (lead-adjusted) aim point, which is what the shooter should use. */
    public double getDistance() {
        return distanceIn;
    }

    public double getAngleDeg() {
        return angleDeg;
    }

    /** Field-computed aim angle before trim and limits (degrees, positive = right). */
    public double getAimAngleDeg() {
        return aimAngleDeg;
    }

    public boolean hasValidPose() {
        return poseValid;
    }

    public double getTargetDeg() {
        return targetDeg;
    }

    public double getErrorDeg() {
        return errorDeg;
    }

    public double getPower() {
        return power;
    }

    public boolean isLimited() {
        return limited;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret", "%s  %s CELL up  %s", mode, upCell, isOnTarget() ? "ON TARGET" : limited ? "AT LIMIT" : "");
        telemetry.addData("Turret angle / target", "%.1f / %.1f (err %.1f)", angleDeg, targetDeg, errorDeg);
        telemetry.addData("Turret trim / power", "%.1f / %.2f", trimDeg, power);
        telemetry.addData("Turret vel / raw", "%.0f deg/s / %.1f deg (%.2f V)", velocityDegPerSec, rawDeg, feedback.getVoltage());
        telemetry.addData("Distance to CELL", "%.1f in (lead %.1f in)", trueDistanceIn, distanceIn);
    }
}
