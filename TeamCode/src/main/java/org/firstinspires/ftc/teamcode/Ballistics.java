package org.firstinspires.ftc.teamcode;

/**
 * Shooter physics: flywheel -> ball exit speed, and ball flight with air drag.
 *
 * Flywheel. A single wheel pressing the ball against a fixed hood: with no slip the ball's
 * centre leaves at half the wheel surface speed (the hood side of the ball is stationary,
 * the wheel side moves at surface speed). How close to "no slip" it gets depends on how hard
 * the ball is compressed: grip = 1 - exp(-compression / FULL_GRIP_COMPRESSION_MM), so 4 mm
 * of compression on a 3 mm scale gives about 74 % of the ideal transfer. EFFICIENCY_TRIM is
 * the one number to fit from the real robot (a chronograph, or the RPM that scores from one
 * measured distance): measured exit speed / predicted.
 *
 * Flight. Point mass with quadratic drag (0.5 rho Cd A v^2) and gravity, integrated in small
 * steps. A shot is defined by horizontal distance d to the CELL opening, the exit angle the
 * hood is set to, and the exit speed that drops the ball through the opening on the way DOWN
 * (solveExitSpeed). ShotTable turns that into RPM per distance and a regression the robot uses.
 *
 * Hood policy. The opening is h = CELL_OPENING_HEIGHT_IN - LAUNCH_HEIGHT_IN above the launch
 * and tipped toward the shooter, so the ball has to arrive descending. For a parabola through
 * the opening that arrives ENTRY_ANGLE_DEG below horizontal, tan(exit) = 2h/d + tan(entry):
 * nearly vertical lobs up close (a ball cannot come down through a point higher than its apex),
 * flatter far away. The hood follows that, clamped to its mechanical range.
 *
 * Units: inches and seconds outside, SI inside the drag term. All values are placeholders
 * to measure: ball mass and diameter, wheel diameter, compression, launch height, hood range.
 */
public final class Ballistics {
    // ---- the ball: POLLEN by default (2.8 in, 24.9 g); setBall() switches to NECTAR ----
    public static double BALL_DIAMETER_IN = Field.POLLEN_DIAMETER_IN;
    public static double BALL_MASS_KG = Field.POLLEN_MASS_KG;
    private static boolean nectar = false;
    public static double DRAG_COEFFICIENT = 0.47;     // sphere
    public static double AIR_DENSITY_KG_M3 = 1.2;
    public static final double GRAVITY_IN_S2 = 386.09;

    // ---- the flywheel ----
    public static double WHEEL_DIAMETER_IN = 4.0;
    /** how far the ball is squeezed between wheel and hood */
    public static double COMPRESSION_MM = 4.0;
    public static double FULL_GRIP_COMPRESSION_MM = 3.0;
    /** ideal ball-centre speed / wheel surface speed for one wheel against a fixed hood */
    public static double IDEAL_TRANSFER = 0.5;
    /** measured exit speed / predicted; fit this on the robot */
    public static double EFFICIENCY_TRIM = 1.0;

    // ---- the launch ----
    /** where the ball leaves the shooter, above the tiles */
    public static double LAUNCH_HEIGHT_IN = 14.0;
    /** how steeply (below horizontal) the ball should arrive at the opening, no-drag design value */
    public static double ENTRY_ANGLE_DEG = 30.0;
    // hood servo calibration: servo position at two known exit angles (measure with a level)
    public static double HOOD_MIN_ANGLE_DEG = 40.0;
    public static double HOOD_MAX_ANGLE_DEG = 80.0;
    public static double HOOD_SERVO_AT_MIN_ANGLE = 0.10;
    public static double HOOD_SERVO_AT_MAX_ANGLE = 0.80;

    public static double FLIGHT_DT_S = 0.002;

    private Ballistics() {
    }

    /** POLLEN or NECTAR in the shooter: the table regenerates for the other ball. */
    public static void setNectar(boolean shootNectar) {
        if (shootNectar == nectar) {
            return;
        }
        nectar = shootNectar;
        BALL_DIAMETER_IN = nectar ? Field.NECTAR_DIAMETER_IN : Field.POLLEN_DIAMETER_IN;
        BALL_MASS_KG = nectar ? Field.NECTAR_MASS_KG : Field.POLLEN_MASS_KG;
        ShotTable.regenerate();
    }

    public static boolean isNectar() {
        return nectar;
    }

    // ------------------------------------------------------------------ flywheel

    /** ball exit speed / wheel surface speed */
    public static double transferRatio() {
        double grip = 1.0 - Math.exp(-COMPRESSION_MM / FULL_GRIP_COMPRESSION_MM);
        return IDEAL_TRANSFER * grip * EFFICIENCY_TRIM;
    }

    public static double wheelSurfaceSpeedInS(double rpm) {
        return rpm / 60.0 * Math.PI * WHEEL_DIAMETER_IN;
    }

    public static double exitSpeedInS(double rpm) {
        return wheelSurfaceSpeedInS(rpm) * transferRatio();
    }

    public static double rpmForExitSpeed(double exitSpeedInS) {
        return exitSpeedInS / transferRatio() / (Math.PI * WHEEL_DIAMETER_IN) * 60.0;
    }

    // ------------------------------------------------------------------ hood

    /** exit angle policy: tan(exit) = 2h/d + tan(entry), clamped to the hood's range */
    public static double exitAngleDeg(double distanceIn) {
        double h = Field.CELL_OPENING_HEIGHT_IN - LAUNCH_HEIGHT_IN;
        double d = Math.max(6.0, distanceIn);
        double angle = Math.toDegrees(Math.atan(2.0 * h / d + Math.tan(Math.toRadians(ENTRY_ANGLE_DEG))));
        return Math.max(HOOD_MIN_ANGLE_DEG, Math.min(HOOD_MAX_ANGLE_DEG, angle));
    }

    public static double hoodServoForAngle(double angleDeg) {
        double t = (angleDeg - HOOD_MIN_ANGLE_DEG) / (HOOD_MAX_ANGLE_DEG - HOOD_MIN_ANGLE_DEG);
        t = Math.max(0.0, Math.min(1.0, t));
        return HOOD_SERVO_AT_MIN_ANGLE + t * (HOOD_SERVO_AT_MAX_ANGLE - HOOD_SERVO_AT_MIN_ANGLE);
    }

    public static double hoodAngleForServo(double position) {
        double t = (position - HOOD_SERVO_AT_MIN_ANGLE) / (HOOD_SERVO_AT_MAX_ANGLE - HOOD_SERVO_AT_MIN_ANGLE);
        return HOOD_MIN_ANGLE_DEG + t * (HOOD_MAX_ANGLE_DEG - HOOD_MIN_ANGLE_DEG);
    }

    // ------------------------------------------------------------------ flight

    /** A trajectory: horizontal distance x and height above the launch point z, per step. */
    public static final class Flight {
        public final double[] t;
        public final double[] x;
        public final double[] z;
        public final int n;
        public final double apexZ;

        Flight(double[] t, double[] x, double[] z, int n, double apexZ) {
            this.t = t;
            this.x = x;
            this.z = z;
            this.n = n;
            this.apexZ = apexZ;
        }

        /** height above launch when descending through horizontal distance d, or NaN */
        public double descendingHeightAt(double d) {
            for (int i = 1; i < n; i++) {
                if (z[i] < z[i - 1] && x[i - 1] <= d && x[i] >= d) {
                    double f = (d - x[i - 1]) / Math.max(1e-9, x[i] - x[i - 1]);
                    return z[i - 1] + f * (z[i] - z[i - 1]);
                }
            }
            return Double.NaN;
        }

        /** time when the ball reaches horizontal distance d (any branch), or NaN */
        public double timeAt(double d) {
            for (int i = 1; i < n; i++) {
                if (x[i - 1] <= d && x[i] >= d) {
                    double f = (d - x[i - 1]) / Math.max(1e-9, x[i] - x[i - 1]);
                    return t[i - 1] + f * (t[i] - t[i - 1]);
                }
            }
            return Double.NaN;
        }
    }

    /** drag acceleration magnitude coefficient in 1/in: a = -k |v| v with v in in/s */
    private static double dragK() {
        double radiusM = BALL_DIAMETER_IN * 0.0254 / 2.0;
        double area = Math.PI * radiusM * radiusM;
        double kSi = 0.5 * AIR_DENSITY_KG_M3 * DRAG_COEFFICIENT * area / BALL_MASS_KG; // 1/m
        return kSi * 0.0254; // 1/in
    }

    /**
     * Fly a ball launched at exitSpeed (in/s) and angle above horizontal until it falls
     * floorBelowLaunchIn below the launch point or passes maxRangeIn.
     */
    public static Flight fly(double exitSpeedInS, double angleDeg, double floorBelowLaunchIn, double maxRangeIn) {
        double k = dragK();
        double dt = FLIGHT_DT_S;
        int cap = (int) (6.0 / dt);
        double[] t = new double[cap];
        double[] x = new double[cap];
        double[] z = new double[cap];
        double vx = exitSpeedInS * Math.cos(Math.toRadians(angleDeg));
        double vz = exitSpeedInS * Math.sin(Math.toRadians(angleDeg));
        double px = 0, pz = 0, time = 0, apex = 0;
        int n = 0;
        while (n < cap) {
            t[n] = time;
            x[n] = px;
            z[n] = pz;
            n++;
            if (pz < -floorBelowLaunchIn || px > maxRangeIn) {
                break;
            }
            double speed = Math.hypot(vx, vz);
            double ax = -k * speed * vx;
            double az = -GRAVITY_IN_S2 - k * speed * vz;
            vx += ax * dt;
            vz += az * dt;
            px += vx * dt;
            pz += vz * dt;
            time += dt;
            apex = Math.max(apex, pz);
        }
        return new Flight(t, x, z, n, apex);
    }

    /**
     * Exit speed (in/s) that brings the ball DOWN through the point (distanceIn, heightAboveLaunchIn),
     * or NaN if no speed does at this angle (too steep / too close).
     */
    public static double solveExitSpeed(double distanceIn, double angleDeg, double heightAboveLaunchIn) {
        double lo = 30.0;
        double hi = 2500.0;
        double floor = LAUNCH_HEIGHT_IN;
        if (descendingError(hi, angleDeg, distanceIn, heightAboveLaunchIn, floor) < 0) {
            return Double.NaN; // even flat out it falls short
        }
        if (descendingError(lo, angleDeg, distanceIn, heightAboveLaunchIn, floor) > 0) {
            return Double.NaN; // even the slowest shot is over the target: too close for this angle
        }
        for (int i = 0; i < 60; i++) {
            double mid = 0.5 * (lo + hi);
            double err = descendingError(mid, angleDeg, distanceIn, heightAboveLaunchIn, floor);
            if (err < 0) {
                lo = mid; // too slow
            } else {
                hi = mid; // too fast
            }
            if (hi - lo < 0.05) {
                break;
            }
        }
        return 0.5 * (lo + hi);
    }

    /**
     * Height error at distance d on the way down: positive = ball is above the target there
     * (too fast), negative = below or never got there (too slow). A ball that is still rising
     * when it passes d and lands beyond it counts as too fast.
     */
    private static double descendingError(double v, double angleDeg, double d, double target, double floor) {
        Flight f = fly(v, angleDeg, floor, 1e6);
        double h = f.descendingHeightAt(d);
        if (!Double.isNaN(h)) {
            return h - target;
        }
        double maxX = f.x[f.n - 1];
        return maxX >= d ? 1e9 : -1e9;
    }

    public static String describe() {
        return String.format("ball %.1f in %.0f g, wheel %.1f in, %.1f mm compression -> transfer %.3f; launch %.1f in, entry %.0f deg, hood %.0f..%.0f deg",
                BALL_DIAMETER_IN, BALL_MASS_KG * 1000, WHEEL_DIAMETER_IN, COMPRESSION_MM, transferRatio(),
                LAUNCH_HEIGHT_IN, ENTRY_ANGLE_DEG, HOOD_MIN_ANGLE_DEG, HOOD_MAX_ANGLE_DEG);
    }
}
