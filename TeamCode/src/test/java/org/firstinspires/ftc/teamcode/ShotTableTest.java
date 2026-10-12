package org.firstinspires.ftc.teamcode;

/** Hardware-free shot calibration checks; run main with assertions implemented below. */
public final class ShotTableTest {
    public static void main(String[] args) {
        tableInterpolationAndInputValidation();
        polynomialUnitsAndRange();
        measuredHoodMapping();
        invalidFitsFailClosed();
        System.out.println("ShotTableTest passed");
    }

    private static void tableInterpolationAndInputValidation() {
        equal(3400.0, ShotTable.rpm(0.0), "table low-end clamp");
        equal(5800.0, ShotTable.rpm(200.0), "table high-end clamp");
        equal(3600.0, ShotTable.rpm(36.0), "RPM midpoint");
        equal(0.25, ShotTable.hood(36.0), "servo midpoint");
        equal(0.33, ShotTable.timeOfFlight(36.0), "time midpoint");
        check(ShotTable.calibrationStatus().contains("UNCALIBRATED"), "guesses must be labeled");
        invalid(ShotTable.hoodAngleDegrees(36.0), "servo positions are not angles");
        for (double distance : new double[] {Double.NaN, Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY, -1.0}) {
            invalid(ShotTable.rpm(distance), "invalid distance RPM");
            invalid(ShotTable.hood(distance), "invalid distance hood");
            invalid(ShotTable.timeOfFlight(distance), "invalid distance flight time");
        }
        double[] savedDistances = ShotTable.DISTANCE_IN;
        double[] savedRPM = ShotTable.RPM;
        ShotTable.DISTANCE_IN = new double[] {24.0, 24.0};
        ShotTable.RPM = new double[] {3400.0, 3500.0};
        invalid(ShotTable.rpm(24.0), "duplicate distance rows");
        ShotTable.DISTANCE_IN = new double[] {48.0, 24.0};
        invalid(ShotTable.rpm(36.0), "out-of-order distances");
        ShotTable.RPM = new double[] {3400.0};
        invalid(ShotTable.rpm(36.0), "mismatched row counts");
        ShotTable.DISTANCE_IN = savedDistances;
        ShotTable.RPM = savedRPM;
    }

    private static void polynomialUnitsAndRange() {
        ShotTable.USE_RPM_REGRESSION = true;
        ShotTable.RPM_COEFFICIENTS = new double[] {3000.0, 10.0, 0.1};
        invalid(ShotTable.rpm(30.0), "fit needs a measured distance interval");
        ShotTable.REGRESSION_MIN_DISTANCE_IN = 24.0;
        ShotTable.REGRESSION_MAX_DISTANCE_IN = 144.0;
        equal(3390.0, ShotTable.rpm(30.0), "ascending powers use inches and output RPM");
        equal(3297.6, ShotTable.rpm(24.0), "lower fit boundary");
        equal(6513.6, ShotTable.rpm(144.0), "upper fit boundary");
        invalid(ShotTable.rpm(23.99), "fit cannot extrapolate below measurements");
        invalid(ShotTable.rpm(144.01), "fit cannot extrapolate above measurements");

        ShotTable.USE_TIME_OF_FLIGHT_REGRESSION = true;
        ShotTable.TIME_OF_FLIGHT_S_COEFFICIENTS = new double[] {0.15, 0.005};
        equal(0.30, ShotTable.timeOfFlight(30.0), "flight regression outputs seconds");
    }

    private static void measuredHoodMapping() {
        ShotTable.USE_HOOD_ANGLE_REGRESSION = true;
        ShotTable.HOOD_ANGLE_DEG_COEFFICIENTS = new double[] {30.0, 0.5};
        invalid(ShotTable.hood(30.0), "angle fit requires measured servo conversion");
        ShotTable.HOOD_CALIBRATION_ANGLE_DEG = new double[] {30.0, 50.0, 70.0};
        ShotTable.HOOD_CALIBRATION_SERVO_POSITION = new double[] {0.1, 0.5, 0.7};
        equal(0.4, ShotTable.hood(30.0), "45 degree fit maps to servo position");
        equal(45.0, ShotTable.hoodAngleDegrees(30.0), "physical angle is degrees");
        equal(0.6, ShotTable.angleToHoodPosition(60.0), "nonlinear linkage second segment");
        equal(60.0, ShotTable.hoodPositionToAngleDegrees(0.6), "inverse mapping");
        invalid(ShotTable.angleToHoodPosition(80.0), "unreachable angle fails closed");
        invalid(ShotTable.hood(100.0), "fit outside measured linkage range fails closed");
        invalid(ShotTable.hoodPositionToAngleDegrees(0.9), "unmapped servo position");

        ShotTable.HOOD_CALIBRATION_SERVO_POSITION = new double[] {0.9, 0.5, 0.3};
        equal(0.6, ShotTable.hood(30.0), "reversed servo linkage");
        equal(60.0, ShotTable.hoodPositionToAngleDegrees(0.4), "inverse reversed linkage");
        ShotTable.HOOD_CALIBRATION_SERVO_POSITION = new double[] {0.9, 0.5, 0.6};
        invalid(ShotTable.hood(30.0), "nonmonotonic linkage is ambiguous");
        ShotTable.HOOD_CALIBRATION_SERVO_POSITION = new double[] {0.1, 0.5, 1.2};
        invalid(ShotTable.hood(30.0), "servo calibration must stay in valid range");
    }

    private static void invalidFitsFailClosed() {
        ShotTable.RPM_COEFFICIENTS = new double[] {Double.NaN};
        invalid(ShotTable.rpm(30.0), "nonfinite coefficient");
        ShotTable.RPM_COEFFICIENTS = new double[] {0.0, Double.MAX_VALUE};
        invalid(ShotTable.rpm(30.0), "overflow is not a shot");
        ShotTable.RPM_COEFFICIENTS = new double[] {-10.0};
        invalid(ShotTable.rpm(30.0), "negative RPM");
        ShotTable.RPM_COEFFICIENTS = null;
        invalid(ShotTable.rpm(30.0), "null coefficients");
        ShotTable.RPM_COEFFICIENTS = new double[] {};
        invalid(ShotTable.rpm(30.0), "empty coefficients");
        ShotTable.TIME_OF_FLIGHT_S_COEFFICIENTS = new double[] {0.0};
        invalid(ShotTable.timeOfFlight(30.0), "zero flight time");
        ShotTable.TIME_OF_FLIGHT_S_COEFFICIENTS = new double[] {-0.3};
        invalid(ShotTable.timeOfFlight(30.0), "negative flight time");
        ShotTable.REGRESSION_MAX_DISTANCE_IN = ShotTable.REGRESSION_MIN_DISTANCE_IN;
        invalid(ShotTable.timeOfFlight(30.0), "empty measured interval");
    }

    private static void equal(double expected, double actual, String context) {
        check(Double.isFinite(actual) && Math.abs(expected - actual) <= 1e-8,
                context + ": expected " + expected + ", got " + actual);
    }

    private static void invalid(double value, String context) {
        check(Double.isNaN(value), context + ": expected NaN, got " + value);
    }

    private static void check(boolean condition, String message) {
        if (!condition) throw new AssertionError(message);
    }
}
