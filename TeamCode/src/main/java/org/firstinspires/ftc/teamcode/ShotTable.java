package org.firstinspires.ftc.teamcode;

/**
 * Shot settings by horizontal distance from the turret pivot to the upward CELL aim point.
 *
 * Generated from Ballistics at startup: for each distance, the hood angle from the exit-angle
 * policy, the exit speed that drops the ball through the CELL opening (Field.CELL_OPENING_HEIGHT_IN)
 * and the flywheel RPM that produces it, plus the time of flight for shoot-on-the-move lead.
 * The RPM samples are then fit with a polynomial regression of degree REGRESSION_DEGREE,
 * rpm = REGRESSION[0] + REGRESSION[1] d + REGRESSION[2] d^2 + ..., which is what rpm()
 * returns (smooth, no interpolation kinks). hood() and timeOfFlight() interpolate the table.
 *
 * On the robot, tune Ballistics (mainly EFFICIENCY_TRIM, the launch height and the hood
 * calibration) with the Shot Tuner and call regenerate(); the whole table and regression
 * follow. Or set USE_REGRESSION = false and hand-edit RPM[] the old way.
 *
 * Solving the table is not cheap - every row bisects for an exit speed, and every trial flies
 * the ball in 2 ms steps - so it is done for BOTH balls when the class loads, and switching
 * between POLLEN and NECTAR during a match (gamepad 2 B) only copies one of them into place.
 * Doing the solve on the button press instead would stall the loop for a fifth of a second on
 * a Control Hub, with the turret and the flywheel frozen for that long, at the exact moment
 * the operator has decided to shoot something different.
 */
public final class ShotTable {
    // the up CELL's mouth is 53.4 in from its wall and 59 in from the alliance wall, so the
    // furthest legal shot (from the far corner of that half) is about 80 in
    public static double[] DISTANCE_IN = {18, 24, 30, 36, 42, 48, 54, 60, 66, 72, 78, 84, 90};
    public static double[] RPM = new double[DISTANCE_IN.length];
    public static double[] HOOD = new double[DISTANCE_IN.length];
    public static double[] TIME_OF_FLIGHT_S = new double[DISTANCE_IN.length];
    /** rpm = sum REGRESSION[k] * d^k; cubic because the near end curves hard (steep hood) */
    public static int REGRESSION_DEGREE = 3;
    public static double[] REGRESSION = new double[REGRESSION_DEGREE + 1];
    public static boolean USE_REGRESSION = true;
    /** largest |table - regression| / table over the samples, for telemetry */
    public static double regressionErrorFraction = 0.0;
    /** distances the physics could not solve at their hood angle (too close / too steep) */
    public static int unsolvedRows = 0;

    /** a solved table for one ball, so switching balls is a copy instead of a solve */
    private static final class Solved {
        final double[] rpm;
        final double[] hood;
        final double[] tof;
        final double[] regression;
        final double errorFraction;
        final int unsolved;

        Solved(double[] rpm, double[] hood, double[] tof, double[] regression,
               double errorFraction, int unsolved) {
            this.rpm = rpm;
            this.hood = hood;
            this.tof = tof;
            this.regression = regression;
            this.errorFraction = errorFraction;
            this.unsolved = unsolved;
        }
    }

    private static Solved pollenTable;
    private static Solved nectarTable;

    static {
        regenerate();
    }

    private ShotTable() {
    }

    /**
     * Rebuild both balls' tables and regressions from the current Ballistics constants, and
     * make the ball now in the shooter the live one. Call this after changing anything in
     * Ballistics (the Shot Tuner does).
     */
    public static void regenerate() {
        double diameter = Ballistics.BALL_DIAMETER_IN;
        double mass = Ballistics.BALL_MASS_KG;
        try {
            Ballistics.BALL_DIAMETER_IN = Field.POLLEN_DIAMETER_IN;
            Ballistics.BALL_MASS_KG = Field.POLLEN_MASS_KG;
            pollenTable = solve();
            Ballistics.BALL_DIAMETER_IN = Field.NECTAR_DIAMETER_IN;
            Ballistics.BALL_MASS_KG = Field.NECTAR_MASS_KG;
            nectarTable = solve();
        } finally {
            Ballistics.BALL_DIAMETER_IN = diameter;
            Ballistics.BALL_MASS_KG = mass;
        }
        apply(Ballistics.isNectar() ? nectarTable : pollenTable);
    }

    /**
     * Point the live table at the ball now in the shooter. Microseconds: nothing is re-solved,
     * which is what makes switching safe to do in the middle of a match.
     */
    public static void selectBall(boolean nectar) {
        if (pollenTable == null || nectarTable == null) {
            regenerate();
            return;
        }
        apply(nectar ? nectarTable : pollenTable);
    }

    private static void apply(Solved table) {
        System.arraycopy(table.rpm, 0, RPM, 0, RPM.length);
        System.arraycopy(table.hood, 0, HOOD, 0, HOOD.length);
        System.arraycopy(table.tof, 0, TIME_OF_FLIGHT_S, 0, TIME_OF_FLIGHT_S.length);
        REGRESSION = table.regression.clone();
        regressionErrorFraction = table.errorFraction;
        unsolvedRows = table.unsolved;
    }

    /** Solve every row for whichever ball is currently set in Ballistics. */
    private static Solved solve() {
        double target = Field.CELL_OPENING_HEIGHT_IN - Ballistics.LAUNCH_HEIGHT_IN;
        double[] rpm = new double[DISTANCE_IN.length];
        double[] hood = new double[DISTANCE_IN.length];
        double[] tof = new double[DISTANCE_IN.length];
        int unsolved = 0;
        double lastGood = Double.NaN;
        for (int i = 0; i < DISTANCE_IN.length; i++) {
            double d = DISTANCE_IN[i];
            double angle = Ballistics.exitAngleDeg(d);
            double v = Ballistics.solveExitSpeed(d, angle, target);
            hood[i] = Ballistics.hoodServoForAngle(angle);
            if (Double.isNaN(v)) {
                unsolved++;
                rpm[i] = Double.isNaN(lastGood) ? 3000.0 : lastGood;
                tof[i] = i > 0 ? tof[i - 1] : 0.5;
                continue;
            }
            rpm[i] = Ballistics.rpmForExitSpeed(v);
            lastGood = rpm[i];
            Ballistics.Flight flight = Ballistics.fly(v, angle, Ballistics.LAUNCH_HEIGHT_IN, d + 50.0);
            double flightTime = flight.timeAt(d);
            tof[i] = Double.isNaN(flightTime) ? 0.5 : flightTime;
        }
        double[] regression = fitRegression(rpm);
        double worst = 0.0;
        for (int i = 0; i < DISTANCE_IN.length; i++) {
            double fit = evaluate(regression, DISTANCE_IN[i]);
            worst = Math.max(worst, Math.abs(fit - rpm[i]) / rpm[i]);
        }
        return new Solved(rpm, hood, tof, regression, worst, unsolved);
    }

    /** least-squares polynomial of degree REGRESSION_DEGREE through (DISTANCE_IN, rpmSamples) */
    private static double[] fitRegression(double[] rpmSamples) {
        int n = REGRESSION_DEGREE + 1;
        double[] coefficients = new double[n];
        // normal equations: sum_j (sum_i d^(k+j)) c_j = sum_i rpm d^k
        double[][] m = new double[n][n + 1];
        for (int k = 0; k < n; k++) {
            for (int j = 0; j < n; j++) {
                double acc = 0;
                for (double d : DISTANCE_IN) {
                    acc += Math.pow(d, k + j);
                }
                m[k][j] = acc;
            }
            double rhs = 0;
            for (int i = 0; i < DISTANCE_IN.length; i++) {
                rhs += rpmSamples[i] * Math.pow(DISTANCE_IN[i], k);
            }
            m[k][n] = rhs;
        }
        // Gauss-Jordan with partial pivoting
        for (int col = 0; col < n; col++) {
            int pivot = col;
            for (int r = col + 1; r < n; r++) {
                if (Math.abs(m[r][col]) > Math.abs(m[pivot][col])) {
                    pivot = r;
                }
            }
            double[] tmp = m[col];
            m[col] = m[pivot];
            m[pivot] = tmp;
            for (int r = 0; r < n; r++) {
                if (r == col) {
                    continue;
                }
                double f = m[r][col] / m[col][col];
                for (int c = col; c <= n; c++) {
                    m[r][c] -= f * m[col][c];
                }
            }
        }
        for (int k = 0; k < n; k++) {
            coefficients[k] = m[k][n] / m[k][k];
        }
        return coefficients;
    }

    /** the fitted polynomial at a distance, clamped to the table's range */
    private static double evaluate(double[] coefficients, double distanceIn) {
        double d = Math.max(DISTANCE_IN[0], Math.min(DISTANCE_IN[DISTANCE_IN.length - 1], distanceIn));
        double rpm = 0.0;
        double power = 1.0;
        for (double coefficient : coefficients) {
            rpm += coefficient * power;
            power *= d;
        }
        return rpm;
    }

    public static double regressionRpm(double distanceIn) {
        return evaluate(REGRESSION, distanceIn);
    }

    public static double rpm(double distanceIn) {
        if (Double.isNaN(distanceIn)) {
            return RPM[0];
        }
        return USE_REGRESSION ? regressionRpm(distanceIn) : interpolate(RPM, distanceIn);
    }

    public static double hood(double distanceIn) {
        return interpolate(HOOD, distanceIn);
    }

    public static double timeOfFlight(double distanceIn) {
        return interpolate(TIME_OF_FLIGHT_S, distanceIn);
    }

    /** exit angle the hood is set to at this distance, degrees above horizontal */
    public static double exitAngleDeg(double distanceIn) {
        return Ballistics.exitAngleDeg(Double.isNaN(distanceIn) ? DISTANCE_IN[0] : distanceIn);
    }

    private static double interpolate(double[] values, double distanceIn) {
        double[] distances = DISTANCE_IN;
        int last = distances.length - 1;
        if (Double.isNaN(distanceIn) || distanceIn <= distances[0]) {
            return values[0];
        }
        if (distanceIn >= distances[last]) {
            return values[last];
        }
        for (int i = 1; i <= last; i++) {
            if (distanceIn <= distances[i]) {
                double t = (distanceIn - distances[i - 1]) / (distances[i] - distances[i - 1]);
                return values[i - 1] + t * (values[i] - values[i - 1]);
            }
        }
        return values[last];
    }

    public static String describe() {
        StringBuilder sb = new StringBuilder();
        sb.append(Ballistics.describe()).append('\n');
        sb.append("rpm =");
        for (int k = 0; k < REGRESSION.length; k++) {
            sb.append(String.format(" %s%.6g d^%d", k > 0 && REGRESSION[k] >= 0 ? "+ " : "", REGRESSION[k], k));
        }
        sb.append(String.format("   (max fit error %.2f%%, unsolved rows %d)%n", regressionErrorFraction * 100, unsolvedRows));
        for (int i = 0; i < DISTANCE_IN.length; i++) {
            sb.append(String.format("  %4.0f in: %5.0f rpm (fit %5.0f)  angle %4.1f deg  hood %.3f  tof %.2f s  exit %.0f in/s%n",
                    DISTANCE_IN[i], RPM[i], regressionRpm(DISTANCE_IN[i]), Ballistics.exitAngleDeg(DISTANCE_IN[i]),
                    HOOD[i], TIME_OF_FLIGHT_S[i], Ballistics.exitSpeedInS(RPM[i])));
        }
        return sb.toString();
    }
}
