package org.firstinspires.ftc.teamcode;

/** Hardware-independent field aiming math. Distances are inches, time seconds, heading CCW radians.
 * Velocity is in FIELD coordinates. Turret angles/rates are clockwise degrees relative to forward.
 * Assumes constant field translation and angular velocity over the prediction interval. */
public final class TurretAim {
    private static final double MIN_RANGE_IN = 1.0;
    private static final double MAX_FLIGHT_TIME_S = 3.0;
    private static final double RATE_STEP_S = 0.001;
    private static final int MAX_ITERATIONS = 96;
    private static final double RANGE_TOLERANCE_IN = 0.000001;

    public interface FlightTimeModel {
        double seconds(double distanceIn);
    }

    public static final class Solution {
        public final boolean valid;
        public final double distanceIn;
        public final double geometricDistanceIn;
        public final double geometricAngleDeg;
        public final double angleDeg;
        public final double rateDegPerSec;
        public final double flightTimeS;

        private Solution(boolean valid, double distance, double geometricDistance,
                         double geometricAngle, double angle, double rate, double flightTime) {
            this.valid = valid;
            distanceIn = distance;
            geometricDistanceIn = geometricDistance;
            geometricAngleDeg = geometricAngle;
            angleDeg = angle;
            rateDegPerSec = rate;
            flightTimeS = flightTime;
        }
    }

    private static final Solution INVALID = new Solution(false, Double.NaN, Double.NaN,
            Double.NaN, Double.NaN, Double.NaN, Double.NaN);

    private TurretAim() { }

    public static Solution solve(double x, double y, double heading, double vx, double vy, double omega,
                                 double targetX, double targetY, double pivotForward, double pivotLeft,
                                 double latencyS, double leadGain, FlightTimeModel flightTime) {
        if (!finite(x) || !finite(y) || !finite(heading) || !finite(vx) || !finite(vy)
                || !finite(omega) || !finite(targetX) || !finite(targetY)
                || !finite(pivotForward) || !finite(pivotLeft) || !finite(latencyS)
                || latencyS < 0 || latencyS > 0.5 || !finite(leadGain) || leadGain < 0
                || leadGain > 2 || (leadGain > 0 && flightTime == null)) return INVALID;

        double offsetX = pivotForward * Math.cos(heading) - pivotLeft * Math.sin(heading);
        double offsetY = pivotForward * Math.sin(heading) + pivotLeft * Math.cos(heading);
        double dx = targetX - x - offsetX;
        double dy = targetY - y - offsetY;
        double geometricRange = Math.hypot(dx, dy);
        if (!finite(geometricRange) || geometricRange < MIN_RANGE_IN) return INVALID;
        double geometricAngle = wrapDeg(Math.toDegrees(heading - Math.atan2(dy, dx)));

        Sample current = sample(x, y, heading, vx, vy, omega, targetX, targetY,
                pivotForward, pivotLeft, latencyS, leadGain, flightTime);
        Sample next = sample(x, y, heading, vx, vy, omega, targetX, targetY,
                pivotForward, pivotLeft, latencyS + RATE_STEP_S, leadGain, flightTime);
        if (current == null || next == null) return INVALID;
        // Differentiate the entire compensated solution: includes changing flight time, rotating
        // pivot velocity, and robot spin. Wrap the difference to avoid a spike at +/-180 degrees.
        double rate = wrapDeg(next.angle - current.angle) / RATE_STEP_S;
        if (!finite(rate)) return INVALID;
        return new Solution(true, current.range, geometricRange, geometricAngle,
                current.angle, rate, current.flightTime);
    }

    private static Sample sample(double x, double y, double heading, double vx, double vy, double omega,
                                 double targetX, double targetY, double forward, double left,
                                 double predictionS, double leadGain, FlightTimeModel model) {
        double releaseHeading = heading + omega * predictionS;
        double offsetX = forward * Math.cos(releaseHeading) - left * Math.sin(releaseHeading);
        double offsetY = forward * Math.sin(releaseHeading) + left * Math.cos(releaseHeading);
        double dx = targetX - (x + vx * predictionS + offsetX);
        double dy = targetY - (y + vy * predictionS + offsetY);
        // Ball inherits the velocity of the pivot, including omega cross pivot offset.
        double pivotVx = vx - omega * offsetY;
        double pivotVy = vy + omega * offsetX;
        double range = Math.hypot(dx, dy);
        if (!finite(range) || range < MIN_RANGE_IN) return null;
        double flight = 0;
        double aimX = dx;
        double aimY = dy;
        boolean converged = leadGain == 0;
        if (leadGain > 0) {
            for (int i = 0; i < MAX_ITERATIONS; i++) {
                flight = model.seconds(range);
                if (!finite(flight) || flight <= 0 || flight > MAX_FLIGHT_TIME_S) return null;
                aimX = dx - pivotVx * flight * leadGain;
                aimY = dy - pivotVy * flight * leadGain;
                double nextRange = Math.hypot(aimX, aimY);
                if (!finite(nextRange) || nextRange < MIN_RANGE_IN) return null;
                if (Math.abs(nextRange - range) <= RANGE_TOLERANCE_IN) {
                    range = nextRange;
                    converged = true;
                    break;
                }
                // Damping prevents an otherwise useful distance-dependent fit oscillating.
                range = (range + nextRange) * 0.5;
            }
        }
        if (!converged) return null;
        double angle = wrapDeg(Math.toDegrees(releaseHeading - Math.atan2(aimY, aimX)));
        return finite(angle) ? new Sample(range, angle, flight) : null;
    }

    private static final class Sample {
        final double range, angle, flightTime;
        Sample(double range, double angle, double flightTime) {
            this.range = range;
            this.angle = angle;
            this.flightTime = flightTime;
        }
    }

    /** Nearest equivalent mechanical angle, with hysteresis at the cable boundary. */
    public static double reachableAngle(double desired, double current, double min, double max,
                                        double hysteresis) {
        if (!finite(desired) || !finite(current) || !finite(min) || !finite(max)
                || min > max || !finite(hysteresis) || hysteresis < 0) return Double.NaN;
        double nearest = desired + 360 * Math.rint((current - desired) / 360);
        if (nearest > max && nearest - max <= hysteresis && current >= max - hysteresis) return max;
        if (nearest < min && min - nearest <= hysteresis && current <= min + hysteresis) return min;
        double firstTurn = Math.ceil((min - desired) / 360);
        double lastTurn = Math.floor((max - desired) / 360);
        if (firstTurn <= lastTurn) {
            double turn = Math.max(firstTurn, Math.min(lastTurn, Math.rint((current - desired) / 360)));
            return desired + 360 * turn;
        }
        // With a travel span <360, either endpoint may be nearer to the desired bearing.
        double minError = Math.abs(wrapDeg(min - desired));
        double maxError = Math.abs(wrapDeg(max - desired));
        return minError < maxError ? min : max;
    }

    public static double wrapDeg(double degrees) {
        double wrapped = degrees % 360;
        if (wrapped >= 180) wrapped -= 360;
        if (wrapped < -180) wrapped += 360;
        return wrapped;
    }

    private static boolean finite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }
}
