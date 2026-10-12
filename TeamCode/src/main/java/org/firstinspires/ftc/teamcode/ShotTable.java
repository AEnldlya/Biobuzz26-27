package org.firstinspires.ftc.teamcode;

/**
 * Shot calibration using horizontal distance in inches from the release point to the aim point.
 * Feed the movement-compensated distance here when shooting while driving.
 *
 * The default table contains STARTING GUESSES ONLY. Replace it with measurements or supply
 * regressions after testing. HOOD contains servo positions, not physical hood angles.
 * Invalid input/configuration returns NaN so callers can inhibit the shot instead of guessing.
 */
public final class ShotTable {
    public static double[] DISTANCE_IN = {24, 48, 72, 96, 120, 144};
    public static double[] RPM = {3400, 3800, 4300, 4800, 5300, 5800};
    public static double[] HOOD = {0.20, 0.30, 0.40, 0.48, 0.56, 0.64};
    public static double[] TIME_OF_FLIGHT_S = {0.28, 0.38, 0.48, 0.56, 0.62, 0.68};
    // Set only after replacing ALL fallback rows with measured values.
    public static boolean TABLE_CALIBRATED = false;

    // Each polynomial is c[0] + c[1]*distanceIn + c[2]*distanceIn^2 + ... .
    // RPM output is flywheel revolutions/minute; angle output is degrees above horizontal;
    // flight-time output is seconds from release until reaching the target.
    public static boolean USE_RPM_REGRESSION = false;
    public static boolean USE_HOOD_ANGLE_REGRESSION = false;
    public static boolean USE_TIME_OF_FLIGHT_REGRESSION = false;
    public static double[] RPM_COEFFICIENTS = {};
    public static double[] HOOD_ANGLE_DEG_COEFFICIENTS = {};
    public static double[] TIME_OF_FLIGHT_S_COEFFICIENTS = {};
    // Supply the measured valid distance interval before enabling any regression.
    // Fits are rejected outside this interval; a polynomial must not extrapolate unchecked.
    public static double REGRESSION_MIN_DISTANCE_IN = Double.NaN;
    public static double REGRESSION_MAX_DISTANCE_IN = Double.NaN;

    // Measured linkage mapping: angle entries strictly increase; servo positions must be
    // strictly monotonic (either direction). Two or more points are required; more points
    // allow a nonlinear linkage to be approximated by piecewise linear interpolation.
    // Do not fill these with a guessed conversion such as servoPosition * 180.
    public static double[] HOOD_CALIBRATION_ANGLE_DEG = {};
    public static double[] HOOD_CALIBRATION_SERVO_POSITION = {};

    private ShotTable() {
    }

    public static double rpm(double distanceIn) {
        double value = USE_RPM_REGRESSION
                ? regression(RPM_COEFFICIENTS, distanceIn)
                : interpolateTable(RPM, distanceIn);
        return finite(value) && value >= 0.0 ? value : Double.NaN;
    }

    /** Returns servo position in [0, 1], including when the fitted output is in degrees. */
    public static double hood(double distanceIn) {
        double value = USE_HOOD_ANGLE_REGRESSION
                ? angleToHoodPosition(regression(HOOD_ANGLE_DEG_COEFFICIENTS, distanceIn))
                : interpolateTable(HOOD, distanceIn);
        return finite(value) && value >= 0.0 && value <= 1.0 ? value : Double.NaN;
    }

    /** Returns the physical angle above horizontal, or NaN until the linkage is calibrated. */
    public static double hoodAngleDegrees(double distanceIn) {
        return hoodPositionToAngleDegrees(hood(distanceIn));
    }

    /** Measured flight time in seconds; this is independent of feeder/release latency. */
    public static double timeOfFlight(double distanceIn) {
        double value = USE_TIME_OF_FLIGHT_REGRESSION
                ? regression(TIME_OF_FLIGHT_S_COEFFICIENTS, distanceIn)
                : interpolateTable(TIME_OF_FLIGHT_S, distanceIn);
        return finite(value) && value > 0.0 ? value : Double.NaN;
    }

    public static double angleToHoodPosition(double angleDeg) {
        if (!validHoodMapping() || !finite(angleDeg)) return Double.NaN;
        double[] angles = HOOD_CALIBRATION_ANGLE_DEG;
        if (angleDeg < angles[0] || angleDeg > angles[angles.length - 1]) return Double.NaN;
        return interpolate(angles, HOOD_CALIBRATION_SERVO_POSITION, angleDeg);
    }

    public static double hoodPositionToAngleDegrees(double position) {
        if (!validHoodMapping() || !finite(position)) return Double.NaN;
        double[] positions = HOOD_CALIBRATION_SERVO_POSITION;
        double[] angles = HOOD_CALIBRATION_ANGLE_DEG;
        int last = positions.length - 1;
        if (position < Math.min(positions[0], positions[last])
                || position > Math.max(positions[0], positions[last])) return Double.NaN;
        for (int i = 1; i <= last; i++) {
            if (position >= Math.min(positions[i - 1], positions[i])
                    && position <= Math.max(positions[i - 1], positions[i])) {
                double t = (position - positions[i - 1]) / (positions[i] - positions[i - 1]);
                return angles[i - 1] + t * (angles[i] - angles[i - 1]);
            }
        }
        return Double.NaN;
    }

    public static String calibrationStatus() {
        boolean usesTable = !USE_RPM_REGRESSION || !USE_HOOD_ANGLE_REGRESSION
                || !USE_TIME_OF_FLIGHT_REGRESSION;
        String sources = "RPM " + (USE_RPM_REGRESSION ? "fit" : "table")
                + ", hood " + (USE_HOOD_ANGLE_REGRESSION ? "angle fit" : "servo table")
                + ", flight " + (USE_TIME_OF_FLIGHT_REGRESSION ? "fit" : "table");
        return (usesTable && !TABLE_CALIBRATED ? "UNCALIBRATED STARTING VALUES: " : "Configured: ")
                + sources;
    }

    private static double regression(double[] coefficients, double distanceIn) {
        if (!validDistance(distanceIn) || !finite(REGRESSION_MIN_DISTANCE_IN)
                || !finite(REGRESSION_MAX_DISTANCE_IN) || REGRESSION_MIN_DISTANCE_IN < 0.0
                || REGRESSION_MAX_DISTANCE_IN <= REGRESSION_MIN_DISTANCE_IN
                || distanceIn < REGRESSION_MIN_DISTANCE_IN || distanceIn > REGRESSION_MAX_DISTANCE_IN
                || coefficients == null || coefficients.length == 0) return Double.NaN;
        double value = 0.0;
        for (int i = coefficients.length - 1; i >= 0; i--) {
            if (!finite(coefficients[i])) return Double.NaN;
            value = value * distanceIn + coefficients[i];
        }
        return finite(value) ? value : Double.NaN;
    }

    private static double interpolateTable(double[] values, double distanceIn) {
        if (!validDistance(distanceIn) || !validSeries(DISTANCE_IN, values)
                || DISTANCE_IN[0] < 0.0) return Double.NaN;
        // Preserve the original table's endpoint clamping while awaiting measured fits.
        return interpolate(DISTANCE_IN, values, distanceIn);
    }

    private static double interpolate(double[] x, double[] y, double value) {
        int last = x.length - 1;
        if (value <= x[0]) return y[0];
        if (value >= x[last]) return y[last];
        for (int i = 1; i <= last; i++) {
            if (value <= x[i]) {
                double t = (value - x[i - 1]) / (x[i] - x[i - 1]);
                return y[i - 1] + t * (y[i] - y[i - 1]);
            }
        }
        return Double.NaN;
    }

    private static boolean validHoodMapping() {
        double[] angles = HOOD_CALIBRATION_ANGLE_DEG;
        double[] positions = HOOD_CALIBRATION_SERVO_POSITION;
        if (!validSeries(angles, positions) || angles.length < 2) return false;
        boolean increasing = positions[positions.length - 1] > positions[0];
        for (int i = 0; i < positions.length; i++) {
            if (positions[i] < 0.0 || positions[i] > 1.0) return false;
            if (i > 0 && (increasing ? positions[i] <= positions[i - 1]
                    : positions[i] >= positions[i - 1])) return false;
        }
        return true;
    }

    private static boolean validSeries(double[] x, double[] y) {
        if (x == null || y == null || x.length == 0 || x.length != y.length) return false;
        for (int i = 0; i < x.length; i++) {
            if (!finite(x[i]) || !finite(y[i]) || (i > 0 && x[i] <= x[i - 1])) return false;
        }
        return true;
    }

    private static boolean validDistance(double distanceIn) {
        return finite(distanceIn) && distanceIn >= 0.0;
    }

    private static boolean finite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }
}
