package org.firstinspires.ftc.teamcode.sim;

import static com.pedropathing.api.Paths.line;
import static org.junit.Assert.assertEquals;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.pedro.PathProfiles;
import org.junit.Test;

/**
 * Pins down how a Pedro 3.0.0 Path interpolates heading.
 *
 * AutoBase depends on a path ENDING at the pose it was given: if it ends at the start heading
 * instead, the camera and the intake point the wrong way at the end of every path, which is
 * invisible on a turret robot until something has to aim.
 *
 * As measured here, Pedro 3.0.0's linear(a, b) holds b at t = 0 and a at t = 1: the arguments
 * read backwards from their names. PathProfiles.straight() passes them in that order, and the
 * last check below is the canary. If a later Pedro release swaps them back, these tests fail
 * and straight() is the single place to fix.
 */
public class PathHeadingTest {
    private static double deg(double rad) {
        return ((Math.toDegrees(rad) % 360) + 360) % 360;
    }

    private static void dump(String name, Path p) {
        System.out.printf("%-34s h(0)=%6.1f h(0.5)=%6.1f h(1)=%6.1f%n", name,
                deg(p.heading(0.0)), deg(p.heading(0.5)), deg(p.heading(1.0)));
    }

    @Test
    public void linearTakesItsArgumentsBackwards() {
        Pose from = new Pose(24, 16, Math.toRadians(0));
        Pose to = new Pose(34, 104, Math.toRadians(218));
        dump("linear(from.h, to.h)", line(from, to).linear(from.heading(), to.heading()));
        dump("linear(to.h, from.h)", line(from, to).linear(to.heading(), from.heading()));
        dump("Interpolator.longLinear(a, b)", line(from, to).heading(Interpolator.longLinear(from.heading(), to.heading())));
        dump("constant(to.h)", line(from, to).constant(to.heading()));
        dump("tangent()", line(from, to).tangent());

        Path naive = line(from, to).linear(from.heading(), to.heading());
        assertEquals("linear's first argument lands at t = 1", 0.0, deg(naive.heading(1.0)), 1.0);
        assertEquals("and its second argument at t = 0", 218.0, deg(naive.heading(0.0)), 1.0);
    }

    @Test
    public void straightEndsAtTheTargetHeading() {
        Pose from = new Pose(24, 16, Math.toRadians(0));
        Pose to = new Pose(34, 104, Math.toRadians(218));
        Path p = PathProfiles.straight(from, to);
        dump("PathProfiles.straight(from, to)", p);
        assertEquals("starts at the robot's heading", 0.0, deg(p.heading(0.0)), 1.0);
        assertEquals("ends at the target heading", 218.0, deg(p.heading(1.0)), 1.0);
        assertEquals("and endPose agrees", 218.0, deg(p.endPose().heading()), 1.0);
        assertEquals("endPose x", 34.0, p.endPose().x(), 1e-6);
        assertEquals("endPose y", 104.0, p.endPose().y(), 1e-6);
    }
}
