package org.firstinspires.ftc.teamcode.sim;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Field;
import org.firstinspires.ftc.teamcode.Turret;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * The turret alone: does it stay on the CELL while the robot spins and drives, using only
 * odometry, two CR servos and two analog wires? And does it shrug off a glitching wire?
 */
public class TurretTrackingSimTest {
    /** a loop longer than this is the machine descheduling us, not a loop rate a robot has */
    private static final double STALL_MS = 15.0;

    private static double p95(List<Double> values) {
        List<Double> sorted = new ArrayList<>(values);
        Collections.sort(sorted);
        return sorted.get((int) Math.floor(0.95 * (sorted.size() - 1)));
    }

    @Test
    public void tracksWhileSpinningAndDriving() throws Exception {
        SimWorld world = new SimWorld(Alliance.RED, 0.0);
        world.robot.setPose(new Pose(40, 40, Math.toRadians(90)));
        Turret turret = new Turret(world.hardwareMap);
        turret.setAlliance(Alliance.RED);
        turret.setUpCell(Field.startingUpCell(Alliance.RED));
        turret.setMode(Turret.Mode.AUTO_AIM);
        double lead = Turret.LEAD_GAIN;
        // measure pure pointing, no shoot-on-the-move lead. Restore it however this exits:
        // LEAD_GAIN is static, so leaking 0 here would silently disarm the lead in every test
        // that runs after this one in the same JVM.
        Turret.LEAD_GAIN = 0.0;
        try {

        List<Double> movingErrors = new ArrayList<>();
        double settleError = Double.NaN;
        long start = System.nanoTime();
        int loops = 0;
        int stalled = 0;
        double worstLoopMs = 0;
        long prevNs = start;
        double t;
        while ((t = (System.nanoTime() - start) / 1e9) < 9.0) {
            // The turret has +-180 cable limits, so the robot must not spin a net full turn
            // (that forces a legitimate 360 unwind). Oscillate instead:
            // 0-2 s: still (settle)          2-3.2 s: spin left while driving forward
            // 3.2-5.6 s: spin right, strafe   5.6-7 s: spin left again   7-9 s: stop
            if (t < 2.0) {
                world.robot.manualDrive(0, 0, 0);
            } else if (t < 3.2) {
                world.robot.manualDrive(0.4, 0, 0.25);
            } else if (t < 5.6) {
                world.robot.manualDrive(0, 0.5, -0.25);
            } else if (t < 7.0) {
                world.robot.manualDrive(-0.3, 0, 0.25);
            } else {
                world.robot.stop();
            }
            world.robot.update();
            turret.update(world.robot.state().pose(), world.robot.state().velocity());
            world.setLabel(String.format("t=%.1f err=%.2f", t, world.turretErrorDeg()));
            if (t > 1.8 && t < 2.0) {
                settleError = Math.abs(world.turretErrorDeg());
            }
            long nowNs = System.nanoTime();
            double loopMs = (nowNs - prevNs) / 1e6;
            prevNs = nowNs;
            worstLoopMs = Math.max(worstLoopMs, loopMs);
            if (t > 2.5) {
                // A loop this long is not one a Control Hub would have taken; it is this
                // machine descheduling the test thread. The turret genuinely lags across such a
                // gap, but that measures the laptop, not the robot code, so the sample is not
                // evidence either way. Count them instead, and fail below if there are enough
                // to mean the run was not worth judging.
                if (loopMs <= STALL_MS) {
                    movingErrors.add(Math.abs(world.turretErrorDeg()));
                } else {
                    stalled++;
                }
            }
            loops++;
            Thread.sleep(5);
        }
        world.report.write("turret_tracking");

        double p95 = p95(movingErrors);
        double max = Collections.max(movingErrors);
        double loopHz = loops / t;
        double stalledFraction = stalled / (double) Math.max(1, stalled + movingErrors.size());
        System.out.printf("turret: settled error %.2f deg, moving p95 %.2f deg, max %.2f deg, "
                        + "spin rate %.0f deg/s, loop %.0f Hz (worst %.0f ms, %d stalled samples "
                        + "dropped = %.1f%%)%n",
                settleError, p95, max, Math.toDegrees(world.robot.maxTurnRadS * 0.25),
                loopHz, worstLoopMs, stalled, stalledFraction * 100);
        // How well a discrete controller tracks a moving target depends on how often it runs, so
        // these numbers only mean something at a realistic loop rate. This sim is real-time, so
        // a loaded machine stretches the loop and the turret lags for reasons that have nothing
        // to do with the robot code. Individual stalled loops are dropped above; if a run is
        // mostly stalls, say so plainly instead of reporting tracking that was never measured.
        assertTrue("machine could not keep up: " + Math.round(loopHz) + " Hz average, worst loop "
                + Math.round(worstLoopMs) + " ms", loopHz >= 100.0);
        assertTrue("too much of this run was the machine descheduling us to judge tracking: "
                + stalled + " of " + (stalled + movingErrors.size()) + " samples", stalledFraction < 0.02);
        assertTrue("did not settle on target: " + settleError, settleError < 1.5);
        assertTrue("tracking error while moving too large (p95 " + p95 + ")", p95 < 4.0);
        assertTrue("tracking error spike (max " + max + ")", max < 8.0);
        } finally {
            Turret.LEAD_GAIN = lead;
        }
    }

    @Test
    public void ignoresAGlitchingWire() throws Exception {
        SimWorld world = new SimWorld(Alliance.RED, 0.0);
        world.robot.setPose(new Pose(40, 40, Math.toRadians(90)));
        Turret turret = new Turret(world.hardwareMap);
        turret.setAlliance(Alliance.RED);
        turret.setUpCell(Field.startingUpCell(Alliance.RED));
        turret.setMode(Turret.Mode.AUTO_AIM);

        long start = System.nanoTime();
        double t;
        double worstDuringGlitch = 0;
        boolean sawDisagree = false;
        while ((t = (System.nanoTime() - start) / 1e9) < 5.0) {
            boolean glitch = t > 2.0 && t < 3.5;
            world.turret.setGlitchWire2(glitch);
            world.robot.update();
            turret.update(world.robot.state().pose(), world.robot.state().velocity());
            if (glitch) {
                worstDuringGlitch = Math.max(worstDuringGlitch, Math.abs(world.turretErrorDeg()));
                sawDisagree |= !turret.wiresAgree();
            }
            Thread.sleep(5);
        }
        System.out.printf("glitch: worst error %.2f deg, disagreement flagged %s, agree now %s%n",
                worstDuringGlitch, sawDisagree, turret.wiresAgree());
        assertTrue("the wire disagreement should have been flagged", sawDisagree);
        assertTrue("a glitching wire moved the turret: " + worstDuringGlitch, worstDuringGlitch < 2.5);
        assertTrue("wires should agree again after the glitch", turret.wiresAgree());
        assertFalse("turret should be on target after the glitch", Math.abs(world.turretErrorDeg()) > 2.0);
    }
}
