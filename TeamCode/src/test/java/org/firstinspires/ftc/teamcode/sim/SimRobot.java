package org.firstinspires.ftc.teamcode.sim;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.utils.Angle;

import org.firstinspires.ftc.teamcode.pedro.PoseHistory;

import java.util.ArrayDeque;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Random;

/**
 * The simulated mecanum robot: Pedro's Drivetrain (takes drive powers) and Localizer (reports
 * the pose) in one object, with the physics in between.
 *
 * Wheel powers are formed from forward / strafe / turn exactly like a mecanum drive, normalized
 * if any wheel would exceed 1, and turned back into a body-frame velocity target. The actual
 * velocity follows that target with a first-order lag and a deceleration limit, and the pose
 * integrates it. Odometry noise is added to what the localizer reports; the true pose stays
 * available for scoring.
 *
 * Physics runs inside update(), which the real Follower calls once per loop, so the world
 * advances exactly once per OpMode loop, in real time.
 */
public class SimRobot implements Drivetrain, Localizer, PoseHistory {
    public double maxForwardInS = 54.968;
    public double maxStrafeInS = 44.004;
    public double maxTurnRadS = Math.toRadians(300.0);
    /** velocity response time constant, s */
    public double tau = 0.12;
    /**
     * Zero-power deceleration while COASTING, in/s^2 and rad/s^2. The motors are in FLOAT while
     * the Follower is following a path (Mecanum only switches to BRAKE for manual teleop
     * powers), so this is the number Pedro's tuning calls "zero power acceleration" and the one
     * Foresight's brake model is built on - Constants uses 68.3 / 79.305. Under BRAKE the
     * motors resist and the stop is quicker than this limit, modelled by the velocity lag
     * alone.
     */
    public double coastDecel = 68.3;
    public double coastTurnDecel = Math.toRadians(900.0);
    /** odometry noise, 1 sigma, inches / radians */
    public double poseNoiseIn = 0.05;
    public double headingNoiseRad = Math.toRadians(0.15);

    // true state
    private double x = 72, y = 72, heading = 0;
    private double vForward = 0, vStrafe = 0, omega = 0;   // body frame
    // commanded, body frame, after wheel normalization
    private double cmdForward = 0, cmdStrafe = 0, cmdTurn = 0;
    private boolean braking = true;
    private DrivePowers lastPowers = DrivePowers.zero();

    private MotionState reported = MotionState.zero();
    private final ArrayDeque<long[]> historyTimes = new ArrayDeque<>();
    private final ArrayDeque<Pose> historyPoses = new ArrayDeque<>();
    /** odometry noise stream; change the seed to re-run the same plan against different luck */
    public static long noiseSeed = 7;
    private final Random random = new Random(noiseSeed);
    private long lastStepNs = 0;
    private Runnable onStep = () -> { };
    private double simTimeS = 0.0;

    /** something to run every physics step (the rest of the world) */
    public void setOnStep(Runnable onStep) {
        this.onStep = onStep;
    }

    // ------------------------------------------------------------------ Drivetrain

    private static double[] wheels(double f, double s, double t) {
        return new double[] {f + s + t, f - s - t, f - s + t, f + s - t}; // fl fr bl br
    }

    @Override
    /**
     * Pedro's boolean is "manual", not "normalize": the Follower passes true only for teleop
     * stick powers and false while following or holding a path. The real Mecanum ALWAYS scales
     * the wheel powers down by max(1, |w|max) (applyDrive), and uses the flag only to pick the
     * zero-power behaviour, which MecanumConfig.manualBrakeMode defaults to BRAKE in manual and
     * FLOAT while following. Model both the same way, or the simulated robot saturates
     * differently from the real one and coasts when the real one would brake.
     */
    public void drive(DrivePowers powers, boolean manual) {
        lastPowers = powers;
        double[] w = wheels(powers.forward(), powers.strafe(), powers.turn());
        double max = 0;
        for (double v : w) {
            max = Math.max(max, Math.abs(v));
        }
        if (max > 1.0) {
            for (int i = 0; i < 4; i++) {
                w[i] /= max;
            }
        }
        cmdForward = (w[0] + w[1] + w[2] + w[3]) / 4.0;
        cmdStrafe = (w[0] - w[1] - w[2] + w[3]) / 4.0;
        cmdTurn = (w[0] - w[1] + w[2] - w[3]) / 4.0;
        braking = manual;
    }

    @Override
    public double maxScaling(DrivePowers powers, DrivePowers direction) {
        double[] p = wheels(powers.forward(), powers.strafe(), powers.turn());
        double[] d = wheels(direction.forward(), direction.strafe(), direction.turn());
        double min = 1.0;
        for (int i = 0; i < 4; i++) {
            if (Math.abs(d[i]) < 1e-9) {
                continue;
            }
            double hi = (1.0 - p[i]) / d[i];
            double lo = (-1.0 - p[i]) / d[i];
            if (hi > 0 && hi < min) {
                min = hi;
            }
            if (lo > 0 && lo < min) {
                min = lo;
            }
        }
        return Math.max(0.0, Math.min(1.0, min));
    }

    @Override
    public void stop() {
        stop(true);
    }

    @Override
    public void stop(boolean brake) {
        cmdForward = cmdStrafe = cmdTurn = 0;
        braking = brake;
        lastPowers = DrivePowers.zero();
    }

    @Override
    public double interpolateVelocity(double forward, double strafe, double angle) {
        return 1.0 / (Math.abs(Math.cos(angle)) / forward + Math.abs(Math.sin(angle)) / strafe);
    }

    // ------------------------------------------------------------------ Localizer

    @Override
    public void setPose(Pose pose) {
        x = pose.x();
        y = pose.y();
        heading = pose.heading();
        vForward = vStrafe = omega = 0;
        historyTimes.clear();
        historyPoses.clear();
        reported = MotionState.ofVelocity(pose, Velocity.zero());
    }

    @Override
    public MotionState state() {
        return reported;
    }

    @Override
    public void reset() {
        setPose(Pose.zero());
    }

    @Override
    public Map<String, Object> debug() {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("truePose", truePose());
        m.put("powers", lastPowers);
        return m;
    }

    /** One physics step, real elapsed time. Called by the Follower every loop. */
    @Override
    public void update() {
        long now = System.nanoTime();
        double dt = lastStepNs == 0 ? 0.01 : Math.min(0.05, (now - lastStepNs) / 1e9);
        lastStepNs = now;
        lastDt = dt;
        step(dt);
        onStep.run();
    }

    private double lastDt = 0.01;

    /** length of the most recent physics step, s */
    public double lastDt() {
        return lastDt;
    }

    /** Advance the physics by dt seconds. */
    public void step(double dt) {
        simTimeS += dt;
        double tf = cmdForward * maxForwardInS;
        double ts = cmdStrafe * maxStrafeInS;
        double tt = cmdTurn * maxTurnRadS;
        // 0 = no limit: under BRAKE the motors resist, so the lag alone stops it
        vForward = approach(vForward, tf, dt, braking ? 0 : coastDecel);
        vStrafe = approach(vStrafe, ts, dt, braking ? 0 : coastDecel);
        omega = approach(omega, tt, dt, braking ? 0 : coastTurnDecel);

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double vx = vForward * cos - vStrafe * sin;
        double vy = vForward * sin + vStrafe * cos;
        x += vx * dt;
        y += vy * dt;
        heading = Angle.normalize(heading + omega * dt);

        Pose noisy = new Pose(x + random.nextGaussian() * poseNoiseIn,
                y + random.nextGaussian() * poseNoiseIn,
                Angle.normalize(heading + random.nextGaussian() * headingNoiseRad));
        reported = MotionState.ofVelocity(noisy, new Velocity(vx, vy, omega));

        long now = System.nanoTime();
        historyTimes.addLast(new long[] {now});
        historyPoses.addLast(noisy);
        while (historyTimes.size() > 400) {
            historyTimes.pollFirst();
            historyPoses.pollFirst();
        }
    }

    /** first-order approach to target, with a deceleration limit when coasting to zero */
    private double approach(double v, double target, double dt, double decelLimit) {
        double next = v + (target - v) * Math.min(1.0, dt / tau);
        if (decelLimit > 0 && target == 0.0) {
            double maxChange = decelLimit * dt;
            double change = next - v;
            if (Math.abs(change) > maxChange) {
                next = v + Math.copySign(maxChange, change);
            }
            if (Math.signum(next) != Math.signum(v)) {
                next = 0.0;
            }
        }
        return next;
    }

    @Override
    public Pose poseAt(long nanoTime) {
        if (historyPoses.isEmpty()) {
            return reported.pose();
        }
        Pose before = null;
        long beforeNs = 0;
        java.util.Iterator<long[]> ti = historyTimes.iterator();
        for (Pose p : historyPoses) {
            long t = ti.next()[0];
            if (t >= nanoTime) {
                if (before == null) {
                    return p;
                }
                double span = t - beforeNs;
                double f = span <= 0 ? 1 : (nanoTime - beforeNs) / span;
                return Pose.interpolate(before, p, Math.max(0, Math.min(1, f)));
            }
            before = p;
            beforeNs = t;
        }
        return reported.pose();
    }

    // ------------------------------------------------------------------ truth for scoring

    public Pose truePose() {
        return new Pose(x, y, heading);
    }

    public Velocity trueFieldVelocity() {
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        return new Velocity(vForward * cos - vStrafe * sin, vForward * sin + vStrafe * cos, omega);
    }

    public double speedInS() {
        return Math.hypot(vForward, vStrafe);
    }

    public double simTimeS() {
        return simTimeS;
    }

    /** Scripted driving for tests that bypass the Follower (body-frame powers). */
    public void manualDrive(double forward, double strafe, double turn) {
        drive(new DrivePowers(forward, strafe, turn), true);
    }
}
