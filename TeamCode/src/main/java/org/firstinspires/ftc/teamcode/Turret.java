package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
 * turret code and the trim buttons. The turret must face forward when the OpMode inits unless
 * an angle is handed over from AUTO with setAngleReference().
 */
public class Turret {
    // ---- hardware (same wiring as the DECODE robot) ----
    public static String MOTOR_NAME = "turretMotor";
    // the turret motor's encoder cable is plugged into the frontRightMotor port
    public static String ENCODER_PORT_NAME = "frontRightMotor";
    public static double TICKS_PER_MOTOR_REV = 1425.1;   // goBILDA 50.9:1 gearmotor
    public static double GEAR_RATIO = 2.59375;           // motor revs per turret rev
    public static double ENCODER_DIRECTION = -1.0;       // makes a right turn read positive
    public static double POWER_DIRECTION = 1.0;          // positive power turns the turret right
    public static double MIN_ANGLE_DEG = -180.0;         // cable limits relative to forward
    public static double MAX_ANGLE_DEG = 180.0;
    // turret pivot relative to the robot's odometry tracking center, inches
    public static double PIVOT_FORWARD_IN = 0.0;
    public static double PIVOT_LEFT_IN = 0.0;

    // ---- PIDF, in power and degrees ----
    public static double kP = 0.007;
    public static double kI = 0.0005;
    public static double kD = 0.0004;
    // power per deg/s: a 117 RPM motor through 2.59375:1 turns the turret ~270 deg/s at full power
    public static double kV = 1.0 / 270.0;
    public static double kS = 0.08;             // static friction kick (the old MIN_POWER)
    public static double I_ZONE_DEG = 5.0;
    public static double MAX_I_POWER = 0.1;
    public static double MAX_POWER = 0.9;
    public static double DEADBAND_DEG = 0.4;    // settled inside this: stop commanding, no chatter
    public static double ON_TARGET_DEG = 1.5;
    public static double ON_TARGET_RATE_DEG_PER_SEC = 8.0;
    public static double VELOCITY_FILTER = 0.5; // weight of the newest turret velocity sample
    // when the target is only this far past a limit, stay pinned instead of swinging 360
    public static double WRAP_HYSTERESIS_DEG = 15.0;
    public static double TRIM_LIMIT_DEG = 45.0;

    // aim where the CELL will be relative to us when the ball arrives (uses
    // ShotTable.TIME_OF_FLIGHT_S); 0 turns shoot-while-moving compensation off
    public static double LEAD_GAIN = 1.0;
    // Measured sensor/control + ball-release delay. Keep 0 until measured, at most 0.5 s.
    public static double RELEASE_LATENCY_S = 0.0;

    public enum Mode {
        AUTO_AIM,
        HOLD_FORWARD,
        MANUAL,
        OFF
    }

    private final DcMotorEx motor;
    private final DcMotorEx encoder;
    private final Battery battery;
    private final PIDFController pidf = new PIDFController(kP, kI, kD, kV, kS);
    private final TurretAim.FlightTimeModel flightTimeModel = ShotTable::timeOfFlight;

    private Mode mode = Mode.OFF;
    private Alliance alliance = Alliance.RED;
    private Field.CellSide upCell = Field.startingUpCell(Alliance.RED);
    private double zeroTicks;
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
    private double geometricAngleDeg = Double.NaN;
    private double flightTimeS = Double.NaN;
    private boolean poseValid = false;
    private boolean limited = false;
    private long lastNs;
    private boolean resetVelocitySample = true;

    public Turret(HardwareMap hardwareMap, Battery battery) {
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        encoder = hardwareMap.get(DcMotorEx.class, ENCODER_PORT_NAME);
        this.battery = battery;

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        writePower(0.0);

        zeroTicks = encoder.getCurrentPosition();
        lastNs = System.nanoTime();
    }

    /** Tell the turret its current angle (e.g. the angle AUTO ended at) instead of forward = 0. */
    public void setAngleReference(double currentAngleDeg) {
        if (!isFinite(currentAngleDeg)) throw new IllegalArgumentException("Turret reference must be finite");
        zeroTicks = encoder.getCurrentPosition() - currentAngleDeg * ticksPerDegree() * ENCODER_DIRECTION;
        angleDeg = currentAngleDeg;
        lastAngleDeg = currentAngleDeg;
        resetTracking();
    }

    /** Clear aiming history after a field-pose correction; does not change turret zero or trim. */
    public void resetTracking() {
        pidf.reset();
        resetVelocitySample = true;
        velocityDegPerSec = 0.0;
        invalidateAim();
        lastNs = System.nanoTime();
        applyPower(0.0);
    }

    private void invalidateAim() {
        poseValid = false;
        limited = false;
        distanceIn = trueDistanceIn = geometricAngleDeg = flightTimeS = Double.NaN;
        aimAngleDeg = targetDeg = errorDeg = Double.NaN;
        aimRateDegPerSec = targetRateDegPerSec = 0.0;
    }

    public void update(Pose pose, Velocity velocity) {
        long now = System.nanoTime();
        double elapsed = (now - lastNs) / 1e9;
        double dt = Math.min(Math.max(elapsed, 0.001), 0.1);
        lastNs = now;

        angleDeg = readAngleDeg();
        if (resetVelocitySample || elapsed > 0.25) {
            velocityDegPerSec = 0.0;
            resetVelocitySample = false;
            pidf.reset();
        } else {
            double weight = Math.max(0.0, Math.min(1.0, VELOCITY_FILTER));
            velocityDegPerSec += weight * ((angleDeg - lastAngleDeg) / Math.max(elapsed, 0.001) - velocityDegPerSec);
        }
        lastAngleDeg = angleDeg;

        invalidateAim();
        poseValid = pose != null && isFinite(pose.x()) && isFinite(pose.y()) && isFinite(pose.heading())
                && velocity != null && isFinite(velocity.vx) && isFinite(velocity.vy) && isFinite(velocity.omega);
        if (poseValid) {
            computeAim(pose, velocity);
        }
        if (!isFinite(angleDeg) || (mode == Mode.AUTO_AIM && !poseValid)) {
            pidf.reset();
            applyPower(0.0);
            return;
        }

        double desired;
        double desiredRate = 0.0;
        switch (mode) {
            case AUTO_AIM:
                desired = aimAngleDeg + trimDeg;
                desiredRate = aimRateDegPerSec;
                break;
            case HOLD_FORWARD:
                desired = 0.0;
                break;
            case MANUAL:
                pidf.reset();
                applyPower(manualPower);
                return;
            default:
                pidf.reset();
                applyPower(0.0);
                return;
        }

        targetDeg = chooseReachable(desired, angleDeg);
        if (!isFinite(targetDeg)) {
            invalidateAim();
            pidf.reset();
            applyPower(0.0);
            return;
        }
        limited = Math.abs(wrapDeg(targetDeg - desired)) > 0.5;
        targetRateDegPerSec = limited ? 0.0 : desiredRate;
        errorDeg = targetDeg - angleDeg;

        boolean settled = Math.abs(errorDeg) < DEADBAND_DEG && Math.abs(targetRateDegPerSec) < 2.0
                && Math.abs(velocityDegPerSec) < 2.0;
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
        Pose cell = Field.cellAimPoint(alliance, upCell);
        TurretAim.Solution solution = TurretAim.solve(pose.x(), pose.y(), pose.heading(),
                velocity.vx, velocity.vy, velocity.omega, cell.x(), cell.y(),
                PIVOT_FORWARD_IN, PIVOT_LEFT_IN, RELEASE_LATENCY_S, LEAD_GAIN, flightTimeModel);
        poseValid = solution.valid;
        if (!poseValid) return;
        trueDistanceIn = solution.geometricDistanceIn;
        geometricAngleDeg = solution.geometricAngleDeg;
        distanceIn = solution.distanceIn;
        aimAngleDeg = solution.angleDeg;
        aimRateDegPerSec = solution.rateDegPerSec;
        flightTimeS = solution.flightTimeS;
    }

    private double chooseReachable(double desired, double current) {
        return TurretAim.reachableAngle(desired, current, MIN_ANGLE_DEG, MAX_ANGLE_DEG, WRAP_HYSTERESIS_DEG);
    }

    private void applyPower(double requested) {
        if (!isFinite(requested) || !isFinite(angleDeg) || !isFinite(POWER_DIRECTION)
                || !isFinite(MIN_ANGLE_DEG) || !isFinite(MAX_ANGLE_DEG) || MIN_ANGLE_DEG > MAX_ANGLE_DEG) {
            requested = 0.0;
        }
        requested = Math.max(-1.0, Math.min(1.0, requested));
        if ((angleDeg >= MAX_ANGLE_DEG && requested > 0.0) || (angleDeg <= MIN_ANGLE_DEG && requested < 0.0)) {
            requested = 0.0;
        }
        power = requested;
        writePower(requested == 0.0 ? 0.0 : Math.signum(POWER_DIRECTION) * requested);
    }

    private void writePower(double value) {
        // skip redundant writes; each one is a hub transaction
        if (Double.isNaN(lastPowerWritten) || Math.abs(value - lastPowerWritten) > 0.005
                || (value == 0.0 && lastPowerWritten != 0.0)) {
            motor.setPower(value);
            lastPowerWritten = value;
        }
    }

    private double readAngleDeg() {
        return (encoder.getCurrentPosition() - zeroTicks) * ENCODER_DIRECTION / ticksPerDegree();
    }

    private static double ticksPerDegree() {
        return TICKS_PER_MOTOR_REV * GEAR_RATIO / 360.0;
    }

    private static double wrapDeg(double degrees) {
        return TurretAim.wrapDeg(degrees);
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
        if (!isFinite(deltaDeg)) return;
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
        return mode == Mode.AUTO_AIM && poseValid && !limited && Math.abs(errorDeg) <= ON_TARGET_DEG
                && Math.abs(targetRateDegPerSec - velocityDegPerSec) <= ON_TARGET_RATE_DEG_PER_SEC;
    }

    /** Distance to the (lead-adjusted) aim point, which is what the shooter should use. */
    public double getDistance() {
        return distanceIn;
    }

    /** Actual pivot-to-CELL distance before motion compensation. */
    public double getGeometricDistance() {
        return trueDistanceIn;
    }

    /** Actual CELL bearing relative to robot forward, clockwise degrees, before lead/trim. */
    public double getGeometricAngleDeg() {
        return geometricAngleDeg;
    }

    public boolean hasValidAim() {
        return poseValid;
    }

    public double getAngleDeg() {
        return angleDeg;
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
        telemetry.addData("Turret", "%s  %s CELL up  %s", mode, upCell,
                !poseValid ? "NO VALID AIM" : isOnTarget() ? "ON TARGET" : limited ? "AT LIMIT" : "TRACKING");
        telemetry.addData("Turret angle / target", "%.1f / %.1f (err %.1f)", angleDeg, targetDeg, errorDeg);
        telemetry.addData("Turret trim / power", "%.1f / %.2f", trimDeg, power);
        telemetry.addData("Distance to CELL", "%.1f in (lead %.1f in)", trueDistanceIn, distanceIn);
        telemetry.addData("CELL bearing / flight", "%.1f deg CW / %.3f s", geometricAngleDeg, flightTimeS);
        telemetry.addData("Turret rate / target", "%.1f / %.1f deg/s", velocityDegPerSec, targetRateDegPerSec);
    }
}
