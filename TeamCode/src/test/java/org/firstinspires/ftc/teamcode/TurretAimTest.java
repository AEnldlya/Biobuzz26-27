package org.firstinspires.ftc.teamcode;

/** Standalone geometry/ballistic checks; runs without Android or FTC hardware. */
public final class TurretAimTest {
    private static int checks;
    private static int failures;

    private TurretAimTest() { }

    public static void main(String[] args) {
        run("stationary cardinal bearings and robot headings", TurretAimTest::cardinalBearings);
        run("rotated turret pivot and geometric telemetry", TurretAimTest::pivotGeometry);
        run("robot spin and angle wrap do not spike feedforward", TurretAimTest::spinAndWrap);
        run("strafe lead and approaching/receding distance", TurretAimTest::translationLead);
        run("rotating pivot inherits tangential velocity", TurretAimTest::rotatingPivot);
        run("latency predicts release position and heading", TurretAimTest::latency);
        run("constant-speed flight-time fit hits the target", TurretAimTest::distanceDependentFlight);
        run("valid fast receding intercept converges", TurretAimTest::fastRecedingIntercept);
        run("rate includes the changing distance-dependent flight time", TurretAimTest::fullDerivative);
        run("zero lead rate agrees with bearing derivative", TurretAimTest::geometricDerivative);
        run("invalid inputs and flight-time fits fail closed", TurretAimTest::invalidInputs);
        run("mechanical equivalents, limits and cable hysteresis", TurretAimTest::mechanicalLimits);
        System.out.println("TurretAim: " + checks + " checks, " + failures + " failed groups");
        if (failures != 0) throw new AssertionError("TurretAim behavioral verification failed");
    }

    private static void cardinalBearings() {
        double[][] cases = {
                {100, 0, 0, 0}, {0, 100, 0, -90}, {-100, 0, 0, -180}, {0, -100, 0, 90},
                {100, 0, 90, 90}, {0, 100, 90, 0}, {100, 0, -90, -90},
                {100, 0, 450, 90}, {100, 100, 45, 0}
        };
        for (double[] c : cases) {
            TurretAim.Solution s = solve(0, 0, Math.toRadians(c[2]), 0, 0, 0,
                    c[0], c[1], 0, 0, 0, 0, null);
            near(s.angleDeg, c[3], 1e-9, "cardinal relative angle");
            near(s.geometricAngleDeg, c[3], 1e-9, "cardinal geometric angle");
            near(s.distanceIn, Math.hypot(c[0], c[1]), 1e-9, "stationary range");
            near(s.rateDegPerSec, 0, 1e-9, "stationary rate");
        }
    }

    private static void pivotGeometry() {
        TurretAim.Solution s = solve(0, 0, Math.PI / 2, 0, 0, 0,
                -5, 110, 10, 5, 0, 0, null);
        near(s.distanceIn, 100, 1e-9, "range starts at the rotated turret pivot");
        near(s.angleDeg, 0, 1e-9, "target straight ahead of pivot");
        s = solve(30, -20, Math.PI, 0, 0, 0, 10, -24, 10, 4, 0, 0, null);
        near(s.distanceIn, 10, 1e-9, "translated pivot range");
        near(s.angleDeg, 0, 1e-9, "translated pivot bearing");
    }

    private static void spinAndWrap() {
        TurretAim.Solution s = solve(0, 0, 0, 0, 0, Math.PI / 2,
                100, 0, 0, 0, 0, 0, null);
        near(s.rateDegPerSec, 90, 1e-8, "CCW robot requires clockwise turret motion");
        s = solve(0, 0, Math.toRadians(179.99), 0, 0, Math.toRadians(40),
                100, 0, 0, 0, 0, 0, null);
        near(s.angleDeg, 179.99, 1e-9, "angle before positive wrap");
        near(s.rateDegPerSec, 40, 1e-8, "rate crosses positive wrap continuously");
        s = solve(0, 0, Math.toRadians(-179.99), 0, 0, Math.toRadians(-40),
                100, 0, 0, 0, 0, 0, null);
        near(s.rateDegPerSec, -40, 1e-8, "rate crosses negative wrap continuously");
        near(TurretAim.wrapDeg(180), -180, 0, "180 convention");
        near(TurretAim.wrapDeg(-540), -180, 0, "negative multiple wraps");
        near(TurretAim.wrapDeg(1081), 1, 0, "positive multiple wraps");
    }

    private static void translationLead() {
        TurretAim.FlightTimeModel halfSecond = distance -> 0.5;
        TurretAim.Solution left = solve(0, 0, 0, 0, 20, 0,
                100, 0, 0, 0, 0, 1, halfSecond);
        near(left.angleDeg, Math.toDegrees(Math.atan(0.1)), 1e-9,
                "left strafe aims right to cancel inherited velocity");
        TurretAim.Solution right = solve(0, 0, 0, 0, -20, 0,
                100, 0, 0, 0, 0, 1, halfSecond);
        near(right.angleDeg, -left.angleDeg, 1e-9, "opposite strafe symmetry");
        near(left.geometricDistanceIn, 100, 1e-9, "uncompensated telemetry range");
        near(left.geometricAngleDeg, 0, 1e-9, "uncompensated telemetry angle");
        near(solve(0, 0, 0, 20, 0, 0, 100, 0, 0, 0, 0, 1, halfSecond).distanceIn,
                90, 1e-9, "approaching needs less relative projectile travel");
        near(solve(0, 0, 0, -20, 0, 0, 100, 0, 0, 0, 0, 1, halfSecond).distanceIn,
                110, 1e-9, "receding needs more relative projectile travel");
        near(solve(0, 0, 0, 20, 0, 0, 100, 0, 0, 0, 0, 0.5, halfSecond).distanceIn,
                95, 1e-9, "lead gain scales inherited translation");
    }

    private static void rotatingPivot() {
        TurretAim.Solution s = solve(0, 0, 0, 0, 0, 1,
                110, 0, 10, 0, 0, 1, distance -> 0.5);
        near(s.angleDeg, Math.toDegrees(Math.atan(0.05)), 1e-9,
                "spin of forward offset supplies leftward launch velocity");
        verifyImpact(s, 0, 0, 0, 0, 0, 1, 110, 0, 10, 0, 0, 1e-8);
        TurretAim.Solution noLead = solve(0, 0, 0, 0, 0, 1,
                110, 0, 10, 0, 0, 0, null);
        near(noLead.angleDeg, 0, 1e-9, "spin lead can be disabled");
    }

    private static void latency() {
        TurretAim.Solution translation = solve(0, 0, 0, 20, 0, 0,
                100, 0, 0, 0, 0.25, 0, null);
        near(translation.distanceIn, 95, 1e-9, "release delay advances translating chassis");
        near(translation.geometricDistanceIn, 100, 1e-9, "telemetry still describes current pose");
        TurretAim.Solution spin = solve(0, 0, 0, 0, 0, Math.PI / 2,
                100, 0, 0, 0, 0.1, 0, null);
        near(spin.angleDeg, 9, 1e-9, "release delay advances heading");
        TurretAim.Solution combined = solve(7, -4, 0.3, 10, 20, 0.5,
                100, 50, 3, -2, 0.08, 1, distance -> 0.4);
        verifyImpact(combined, 7, -4, 0.3, 10, 20, 0.5,
                100, 50, 3, -2, 0.08, 1e-8);
    }

    private static void distanceDependentFlight() {
        // Constant relative projectile speed has an independently solvable quadratic intercept.
        double[][] cases = {
                {0, 0, 0, 25, 15, 0, 120, 40, 0, 0, 0},
                {-10, 7, 1.2, -20, 30, -0.7, 95, -42, 5, -3, 0.09},
                {40, 25, -2, 40, -15, 1.1, -50, 90, -6, 4, 0.2}
        };
        for (double[] c : cases) {
            TurretAim.Solution s = solve(c[0], c[1], c[2], c[3], c[4], c[5],
                    c[6], c[7], c[8], c[9], c[10], 1, distance -> distance / 150);
            double[] expected = analyticIntercept(c[0], c[1], c[2], c[3], c[4], c[5],
                    c[6], c[7], c[8], c[9], c[10], 150);
            near(s.flightTimeS, expected[0], 2e-5, "quadratic intercept time");
            near(s.distanceIn, expected[0] * 150, 0.003, "quadratic intercept distance");
            angleNear(s.angleDeg, expected[1], 0.002, "quadratic intercept bearing");
            // Launch at 150 inches/s relative to the pivot, then integrate world motion.
            verifyImpactAtSpeed(s, 150, c[0], c[1], c[2], c[3], c[4], c[5],
                    c[6], c[7], c[8], c[9], c[10], 0.004);
        }
    }

    private static void fastRecedingIntercept() {
        TurretAim.Solution s = solve(0, 0, 0, -60, 0, 0,
                100, 0, 0, 0, 0, 1, distance -> distance / 100);
        near(s.flightTimeS, 2.5, 2e-5, "100 inches at net closing speed 40 inches/s");
        near(s.distanceIn, 250, 0.003, "relative projectile travel at 100 inches/s");
        verifyImpactAtSpeed(s, 100, 0, 0, 0, -60, 0, 0,
                100, 0, 0, 0, 0, 0.004);
    }

    private static void fullDerivative() {
        double x = -12, y = 3, heading = 0.7, vx = 32, vy = -17, omega = 0.9;
        double tx = 120, ty = 55, forward = 8, left = -3, latency = 0.07;
        TurretAim.Solution s = solve(x, y, heading, vx, vy, omega,
                tx, ty, forward, left, latency, 1, distance -> distance / 140);
        double dt = 1e-5;
        double[] before = analyticIntercept(x - vx * dt, y - vy * dt, heading - omega * dt,
                vx, vy, omega, tx, ty, forward, left, latency, 140);
        double[] after = analyticIntercept(x + vx * dt, y + vy * dt, heading + omega * dt,
                vx, vy, omega, tx, ty, forward, left, latency, 140);
        double expectedRate = Math.toDegrees(Math.atan2(
                Math.sin(Math.toRadians(after[1] - before[1])),
                Math.cos(Math.toRadians(after[1] - before[1])))) / (2 * dt);
        near(s.rateDegPerSec, expectedRate, 0.04,
                "feedforward differentiates the full ballistic intercept");
    }

    private static void geometricDerivative() {
        double x = 10, y = -20, vx = 17, vy = -8, omega = -0.4, latency = 0.12;
        double dx = 120 - x - vx * latency, dy = 90 - y - vy * latency;
        double rate = Math.toDegrees(omega + (dx * vy - dy * vx) / (dx * dx + dy * dy));
        TurretAim.Solution s = solve(x, y, Math.toRadians(30), vx, vy, omega,
                120, 90, 0, 0, latency, 0, null);
        near(s.rateDegPerSec, rate, 0.002, "analytic angular velocity of the line of sight");
    }

    private static void invalidInputs() {
        check(!TurretAim.solve(Double.NaN, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, null).valid,
                "NaN position rejected");
        check(!TurretAim.solve(0, 0, 0, Double.POSITIVE_INFINITY, 0, 0,
                100, 0, 0, 0, 0, 0, null).valid, "infinite velocity rejected");
        check(!TurretAim.solve(0, 0, 0, 0, 0, 0, 100, 0, 0, 0, -0.01, 0, null).valid,
                "negative latency rejected");
        check(!TurretAim.solve(0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0.501, 0, null).valid,
                "excessive latency rejected");
        check(!TurretAim.solve(0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, -0.1, null).valid,
                "negative lead rejected");
        check(!TurretAim.solve(0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 2.01, d -> 1).valid,
                "excessive lead rejected");
        check(!TurretAim.solve(0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 1, null).valid,
                "missing required flight model rejected");
        for (double invalidTime : new double[] {Double.NaN, Double.POSITIVE_INFINITY, 0, -1, 3.01}) {
            check(!TurretAim.solve(0, 0, 0, 0, 0, 0, 100, 0, 0, 0,
                    0, 1, d -> invalidTime).valid, "invalid flight time rejected");
        }
        check(TurretAim.solve(0, 0, 0, 0, 0, 0, 100, 0, 0, 0,
                0, 1, d -> 3).valid, "flight-time upper boundary accepted");
        check(TurretAim.solve(0, 0, 0, 0, 0, 0, 100, 0, 0, 0,
                0, 0, d -> Double.NaN).valid, "disabled lead does not evaluate an unused model");
        check(!TurretAim.solve(0, 0, 0, 0, 0, 0, 0.99, 0, 0, 0, 0, 0, null).valid,
                "near-zero target range rejected");
        check(!TurretAim.solve(0, 0, 0, 0, 0, 0, 10, 0, 10, 0, 0, 0, null).valid,
                "target coincident with pivot rejected");
        check(!TurretAim.solve(0, 0, 0, 100, 0, 0, 10, 0, 0, 0, 0.1, 0, null).valid,
                "predicted pivot coincident with target rejected");
        check(!TurretAim.solve(0, 0, 0, 100, 0, 0, 100, 0, 0, 0, 0, 1, d -> 1).valid,
                "zero compensated range rejected");
        check(!TurretAim.solve(0, 0, 0, -100, 0, 0, 100, 0, 0, 0,
                0, 1, d -> d / 100).valid, "projectile unable to close range rejected");
        check(!TurretAim.solve(0, 0, 0, 0, 0, 0, Double.MAX_VALUE, Double.MAX_VALUE,
                0, 0, 0, 0, null).valid, "overflowing range rejected");
    }

    private static void mechanicalLimits() {
        near(TurretAim.reachableAngle(-170, 175, -200, 200, 2), 190, 0,
                "nearest physically equivalent angle chosen");
        near(TurretAim.reachableAngle(170, -175, -200, 200, 2), -190, 0,
                "opposite equivalent angle chosen");
        near(TurretAim.reachableAngle(20, 735, -1080, 1080, 2), 740, 0,
                "multi-turn travel chooses nearest revolution");
        near(TurretAim.reachableAngle(-179, 179, -180, 180, 2), 180, 0,
                "upper cable boundary holds inside hysteresis");
        near(TurretAim.reachableAngle(179, -179, -180, 180, 2), -180, 0,
                "lower cable boundary holds inside hysteresis");
        near(TurretAim.reachableAngle(-175, 179, -180, 180, 2), -175, 0,
                "crossing outside hysteresis permits rewind");
        near(TurretAim.reachableAngle(-179, 100, -180, 180, 2), -179, 0,
                "hysteresis requires current turret near boundary");
        near(TurretAim.reachableAngle(140, 20, -45, 60, 2), 60, 0,
                "narrow range clamps to closest bearing endpoint");
        near(TurretAim.reachableAngle(-140, 20, -45, 60, 2), -45, 0,
                "narrow range selects lower endpoint");
        near(TurretAim.reachableAngle(-179, -20, -30, 100, 2), 100, 0,
                "narrow-range endpoint comparison uses circular bearing error");
        near(TurretAim.reachableAngle(-179, 181, 170, 190, 0), 181, 0,
                "narrow mechanical interval may cross the normalized angle boundary");
        near(TurretAim.reachableAngle(30, 0, 15, 15, 2), 15, 0,
                "zero travel span remains at its only permitted angle");
        check(Double.isNaN(TurretAim.reachableAngle(0, 0, 90, -90, 2)),
                "reversed limits rejected");
        check(Double.isNaN(TurretAim.reachableAngle(0, 0, -90, 90, -1)),
                "negative hysteresis rejected");
        check(Double.isNaN(TurretAim.reachableAngle(Double.NaN, 0, -90, 90, 2)),
                "nonfinite desired angle rejected");
    }

    private static TurretAim.Solution solve(double x, double y, double h, double vx, double vy,
                                           double omega, double tx, double ty, double forward,
                                           double left, double latency, double gain,
                                           TurretAim.FlightTimeModel model) {
        TurretAim.Solution s = TurretAim.solve(x, y, h, vx, vy, omega,
                tx, ty, forward, left, latency, gain, model);
        check(s.valid, "expected a valid aiming solution");
        return s;
    }

    /** Positive root of |target - release - pivotVelocity * t| = projectileSpeed * t. */
    private static double[] analyticIntercept(double x, double y, double heading, double vx,
                                              double vy, double omega, double tx, double ty,
                                              double forward, double left, double latency,
                                              double speed) {
        double h = heading + omega * latency;
        double ox = forward * Math.cos(h) - left * Math.sin(h);
        double oy = forward * Math.sin(h) + left * Math.cos(h);
        double rx = tx - x - vx * latency - ox, ry = ty - y - vy * latency - oy;
        double ux = vx - omega * oy, uy = vy + omega * ox;
        double a = speed * speed - ux * ux - uy * uy;
        double dot = rx * ux + ry * uy;
        double t = (-dot + Math.sqrt(dot * dot + a * (rx * rx + ry * ry))) / a;
        double bearing = Math.toDegrees(h - Math.atan2(ry - uy * t, rx - ux * t));
        return new double[] {t, bearing};
    }

    private static void verifyImpact(TurretAim.Solution s, double x, double y, double heading,
                                     double vx, double vy, double omega, double tx, double ty,
                                     double forward, double left, double latency, double tolerance) {
        verifyImpactAtSpeed(s, s.distanceIn / s.flightTimeS, x, y, heading, vx, vy, omega,
                tx, ty, forward, left, latency, tolerance);
    }

    private static void verifyImpactAtSpeed(TurretAim.Solution s, double speed, double x, double y,
                                            double heading, double vx, double vy, double omega,
                                            double tx, double ty, double forward, double left,
                                            double latency, double tolerance) {
        // Integrate a freely moving projectile after release; the chassis keeps rotating but the
        // ball does not keep circling it. This checks the meaning of the returned aim and range.
        double h = heading + omega * latency;
        double ox = forward * Math.cos(h) - left * Math.sin(h);
        double oy = forward * Math.sin(h) + left * Math.cos(h);
        double projectileHeading = h - Math.toRadians(s.angleDeg);
        double ballVx = vx - omega * oy + speed * Math.cos(projectileHeading);
        double ballVy = vy + omega * ox + speed * Math.sin(projectileHeading);
        double impactX = x + vx * latency + ox + ballVx * s.flightTimeS;
        double impactY = y + vy * latency + oy + ballVy * s.flightTimeS;
        near(Math.hypot(impactX - tx, impactY - ty), 0, tolerance, "projectile impact error");
    }

    private static void angleNear(double actual, double expected, double tolerance, String message) {
        double difference = Math.toDegrees(Math.atan2(Math.sin(Math.toRadians(actual - expected)),
                Math.cos(Math.toRadians(actual - expected))));
        near(difference, 0, tolerance, message);
    }

    private static void near(double actual, double expected, double tolerance, String message) {
        check(Double.isFinite(actual) && Math.abs(actual - expected) <= tolerance,
                message + ": expected " + expected + ", got " + actual);
    }

    private static void check(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }

    private static void run(String name, Runnable test) {
        try {
            test.run();
            System.out.println("PASS " + name);
        } catch (AssertionError error) {
            failures++;
            System.err.println("FAIL " + name + ": " + error.getMessage());
        }
    }
}
